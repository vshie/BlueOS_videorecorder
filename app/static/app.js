// Video recorder frontend functionality
let recordingTimer;

async function updateStatus() {
    try {
        const response = await fetch('/status');
        const data = await response.json();
        const statusElement = document.getElementById("recordingStatus");
        
        if (data.recording) {
            statusElement.textContent = "Recording...";
            // Start or update timer if recording
            if (data.start_time) {
                updateRecordingDuration(new Date(data.start_time));
            }
        } else {
            statusElement.textContent = "Stopped";
            if (recordingTimer) {
                clearInterval(recordingTimer);
                recordingTimer = null;
            }
        }
    } catch (error) {
        console.error("Error fetching status:", error);
    }
}

function updateRecordingDuration(startTime) {
    if (!recordingTimer) {
        recordingTimer = setInterval(() => {
            const now = new Date();
            const diff = now - startTime;
            const hours = Math.floor(diff / 3600000);
            const minutes = Math.floor((diff % 3600000) / 60000);
            const seconds = Math.floor((diff % 60000) / 1000);
            const timeString = `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
            document.getElementById("recordingDuration").textContent = `Recording Time: ${timeString}`;
        }, 1000);
    }
}

async function startRecording() {
    try {
        const splitDuration = document.getElementById("splitDuration").value;
        await fetch(`/start?split_duration=${splitDuration}`);
        updateStatus();
    } catch (error) {
        console.error("Error starting recording:", error);
        document.getElementById("recordingStatus").textContent = "Error starting recording";
    }
}

async function stopRecording() {
    try {
        await fetch('/stop');
        if (recordingTimer) {
            clearInterval(recordingTimer);
            recordingTimer = null;
        }
        document.getElementById("recordingDuration").textContent = "";
        updateStatus();
        listVideos();
    } catch (error) {
        console.error("Error stopping recording:", error);
        document.getElementById("recordingStatus").textContent = "Error stopping recording";
    }
}

async function listVideos() {
    try {
        const response = await fetch('/list');
        const data = await response.json();
        const videoList = document.getElementById("videoList");
        videoList.innerHTML = '';
        data.videos.forEach(video => {
            const li = document.createElement('li');
            const a = document.createElement('a');
            a.href = `/download/${video}`;
            a.textContent = video;
            li.appendChild(a);
            videoList.appendChild(li);
        });
    } catch (error) {
        console.error("Error listing videos:", error);
    }
}

// Initial status and video list load
updateStatus();
listVideos();

// Refresh status and video list periodically
setInterval(updateStatus, 1000);
setInterval(listVideos, 5000);