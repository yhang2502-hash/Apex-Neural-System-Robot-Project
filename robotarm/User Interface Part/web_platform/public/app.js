const logEl = document.querySelector("#log");
const serverStatusEl = document.querySelector("#serverStatus");
const buttons = Array.from(document.querySelectorAll("[data-cmd]"));
const refreshHistoryBtn = document.querySelector("#refreshHistory");

function appendLog(message, data) {
  const time = new Date().toLocaleTimeString();
  const extra = data ? `\n${JSON.stringify(data, null, 2)}` : "";
  logEl.textContent = `[${time}] ${message}${extra}\n\n` + logEl.textContent;
}

function setButtonsDisabled(disabled) {
  buttons.forEach((btn) => {
    btn.disabled = disabled;
  });
}

async function checkHealth() {
  try {
    const res = await fetch("/api/health");
    const data = await res.json();

    if (data.ok && data.favoriot_device_configured && data.favoriot_key_configured) {
      serverStatusEl.textContent = "Ready";
      appendLog("Backend ready. Favoriot configuration detected.", {
        device: data.favoriot_device,
        token_header: data.favoriot_token_header
      });
    } else {
      serverStatusEl.textContent = "Config missing";
      appendLog("Server is running, but .env is missing Favoriot details.", data);
    }
  } catch (err) {
    serverStatusEl.textContent = "Offline";
    appendLog("Cannot reach backend server.", { error: err.message });
  }
}

async function sendCommand(cmd) {
  setButtonsDisabled(true);
  appendLog(`Sending command: ${cmd}`);

  try {
    const res = await fetch("/api/command", {
      method: "POST",
      headers: {
        "Content-Type": "application/json"
      },
      body: JSON.stringify({ cmd })
    });

    const data = await res.json();

    if (!res.ok || !data.ok) {
      appendLog(`Command failed: ${cmd}`, data);
      return;
    }

    appendLog(`Command sent successfully: ${cmd}`, data.record);
  } catch (err) {
    appendLog(`Command error: ${cmd}`, { error: err.message });
  } finally {
    setButtonsDisabled(false);
  }
}

async function refreshHistory() {
  try {
    const res = await fetch("/api/history");
    const data = await res.json();
    appendLog("Latest backend command history", data);
  } catch (err) {
    appendLog("Failed to load history", { error: err.message });
  }
}

buttons.forEach((btn) => {
  btn.addEventListener("click", () => {
    const cmd = btn.dataset.cmd;
    sendCommand(cmd);
  });
});

if (refreshHistoryBtn) {
  refreshHistoryBtn.addEventListener("click", refreshHistory);
}

checkHealth();