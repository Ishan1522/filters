use std::sync::Arc;
use std::thread;
use std::time::Duration;

use crossbeam_channel::{unbounded, Receiver, Sender};
use tungstenite::{client::IntoClientRequest, http::HeaderValue, Message};
use url::Url;

use crate::io::nt4_live::LiveStore;
use crate::io::nt4_messages::{
    parse_binary_frame, parse_server_messages, ClientMessage, ServerMessage, SubscribeOptions,
    SubscribeParams,
};

/// Status messages from the network thread to the UI thread, displayed in
/// the live panel. We deliberately don't include detailed error types — the
/// user just needs "connected", "disconnected", or "error: <reason>".
#[derive(Debug, Clone)]
pub enum NtStatus {
    Connecting,
    Connected,
    Disconnected,
    Error(String),
}

/// Commands from the UI thread to the network thread.
#[derive(Debug)]
pub enum NtCommand {
    Disconnect,
}

/// Handle to a running NT4 client. The UI holds this; dropping it does NOT
/// stop the network thread (we'd need a join handle for that). Stopping is
/// explicit via the disconnect command, which lets us cleanly close the
/// WebSocket before the thread exits.
pub struct NtClient {
    pub status_rx: Receiver<NtStatus>,
    pub command_tx: Sender<NtCommand>,
    pub store: Arc<LiveStore>,
}

impl NtClient {
    /// Connect to `ws://host:5810/nt/wpifilter`. The default NT4 port is
    /// 5810, the path component after `/nt/` is the client name (the server
    /// uses it for logging — students sometimes look for it).
    pub fn connect(host: &str) -> Self {
        let url = format!("ws://{host}:5810/nt/wpifilter");
        let (status_tx, status_rx)  = unbounded();
        let (command_tx, command_rx) = unbounded();
        let store = LiveStore::new();
        let store_clone = Arc::clone(&store);

        thread::spawn(move || {
            run_network_thread(url, store_clone, status_tx, command_rx);
        });

        Self { status_rx, command_tx, store }
    }

    pub fn disconnect(&self) {
        let _ = self.command_tx.send(NtCommand::Disconnect);
    }
}

fn run_network_thread(
    url:        String,
    store:      Arc<LiveStore>,
    status_tx:  Sender<NtStatus>,
    command_rx: Receiver<NtCommand>,
) {
    let _ = status_tx.send(NtStatus::Connecting);

    // Build a WebSocket request with the NT4 subprotocol header. The server
    // will reject the connection without it.
    let mut request = match Url::parse(&url).and_then(|u| Ok(u)) {
        Ok(u) => match u.as_str().into_client_request() {
            Ok(r) => r,
            Err(e) => {
                let _ = status_tx.send(NtStatus::Error(format!("bad url: {e}")));
                return;
            }
        },
        Err(e) => {
            let _ = status_tx.send(NtStatus::Error(format!("url parse: {e}")));
            return;
        }
    };
    request.headers_mut().insert(
        "Sec-WebSocket-Protocol",
        HeaderValue::from_static("networktables.first.wpi.edu"),
    );

    let mut socket = match tungstenite::connect(request) {
        Ok((sock, _resp)) => sock,
        Err(e) => {
            let _ = status_tx.send(NtStatus::Error(format!("connect failed: {e}")));
            return;
        }
    };
    let _ = status_tx.send(NtStatus::Connected);

    // Subscribe to everything. The empty-prefix subscription is the canonical
    // "give me all topics" pattern in NT4.
    let subscribe = ClientMessage::Subscribe(SubscribeParams {
        topics:  vec!["".to_string()],
        subuid:  1,
        options: SubscribeOptions {
            prefix:   true,
            periodic: 0.01,  // 10ms throttle — fine for visualization
            all:      false,
        },
    });
    let sub_text = serde_json::to_string(&[subscribe]).unwrap();
    if let Err(e) = socket.send(Message::Text(sub_text.into())) {
        let _ = status_tx.send(NtStatus::Error(format!("subscribe failed: {e}")));
        return;
    }

    // Set the underlying TCP read to non-blocking-ish via a short timeout,
    // so we can periodically check the command channel without spinning.
    if let tungstenite::stream::MaybeTlsStream::Plain(s) = socket.get_ref() {
        let _ = s.set_read_timeout(Some(Duration::from_millis(50)));
    }

    loop {
        // 1. Drain any pending commands.
        if let Ok(cmd) = command_rx.try_recv() {
            match cmd {
                NtCommand::Disconnect => break,
            }
        }

        // 2. Try to read one frame. A timeout returns Err(Io(WouldBlock));
        //    we treat that as "no data this tick" and loop.
        match socket.read() {
            Ok(Message::Text(text)) => {
                handle_text(&store, &text);
            }
            Ok(Message::Binary(bytes)) => {
                handle_binary(&store, &bytes);
            }
            Ok(Message::Close(_)) => {
                let _ = status_tx.send(NtStatus::Disconnected);
                break;
            }
            Ok(_) => {} // Ping/Pong/Frame — handled internally by tungstenite
            Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock
                                          || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                // Normal "no data right now" — loop and try again.
                continue;
            }
            Err(e) => {
                let _ = status_tx.send(NtStatus::Error(format!("read: {e}")));
                break;
            }
        }
    }

    let _ = socket.close(None);
}

fn handle_text(store: &LiveStore, text: &str) {
    for msg in parse_server_messages(text) {
        match msg {
            ServerMessage::Announce(a) => {
                store.announce(a.id, a.name, a.type_str);
            }
            ServerMessage::Unannounce(u) => {
                store.unannounce(u.id);
            }
            ServerMessage::Other => {}
        }
    }
}

fn handle_binary(store: &LiveStore, bytes: &[u8]) {
    for update in parse_binary_frame(bytes) {
        store.push_value(update.topic_id, update.timestamp_us, update.value);
    }
}