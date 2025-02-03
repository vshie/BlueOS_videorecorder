// Global variables
let recordingTimer = null;
let isRecording = false;

async function updateStatus() {
    try {
        const response = await fetch('/status');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();
        const statusElement = document.getElementById("recordingStatus");
        const durationElement = document.getElementById("recordingDuration");
        
        isRecording = data.recording;
        
        if (isRecording) {
            statusElement.textContent = "Recording";
            statusElement.style.color = "red";
            if (data.start_time) {
                if (!recordingTimer) {
                    updateRecordingDuration(new Date(data.start_time));
                }
            }
        } else {
            statusElement.textContent = "Stopped";
            statusElement.style.color = "black";
            if (recordingTimer) {
                clearInterval(recordingTimer);
                recordingTimer = null;
                durationElement.textContent = "00:00:00";
            }
        }
    } catch (error) {
        console.error("Error updating status:", error);
    }
}

function updateRecordingDuration(startTime) {
    if (recordingTimer) {
        clearInterval(recordingTimer);
    }
    
    function updateDuration() {
        if (!isRecording) {
            clearInterval(recordingTimer);
            recordingTimer = null;
            return;
        }
        const now = new Date();
        const duration = Math.floor((now - startTime) / 1000);
        const hours = Math.floor(duration / 3600);
        const minutes = Math.floor((duration % 3600) / 60);
        const seconds = duration % 60;
        document.getElementById("recordingDuration").textContent = 
            `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    }
    
    updateDuration();
    recordingTimer = setInterval(updateDuration, 1000);
}

async function startRecording() {
    try {
        const splitDuration = document.getElementById("splitDuration").value;
        const response = await fetch(`/start?split_duration=${splitDuration}`, {
            method: 'GET'
        });
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        if (data.success) {
            document.getElementById("errorMessage").style.display = "none";
            await updateStatus(); // Immediately update status
        } else {
            throw new Error(data.message || "Failed to start recording");
        }
    } catch (error) {
        console.error("Error starting recording:", error);
        const errorMsg = document.getElementById("errorMessage");
        errorMsg.textContent = error.message || "Error starting recording";
        errorMsg.style.display = "block";
    }
}

async function stopRecording() {
    try {
        const response = await fetch('/stop', {
            method: 'GET'
        });
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        if (data.success) {
            document.getElementById("errorMessage").style.display = "none";
            await updateStatus(); // Immediately update status
            await listVideos(); // Refresh video list
        } else {
            throw new Error(data.message || "Failed to stop recording");
        }
    } catch (error) {
        console.error("Error stopping recording:", error);
        const errorMsg = document.getElementById("errorMessage");
        errorMsg.textContent = error.message || "Error stopping recording";
        errorMsg.style.display = "block";
    }
}

async function listVideos() {
    try {
        const response = await fetch('/list');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        const videoList = document.getElementById("videoList");
        videoList.innerHTML = '';
        
        if (data.videos && data.videos.length > 0) {
            data.videos.forEach(video => {
                const li = document.createElement('li');
                const a = document.createElement('a');
                a.href = `/download/${video}`;
                a.textContent = video;
                li.appendChild(a);
                videoList.appendChild(li);
            });
            document.getElementById("errorMessage").style.display = "none";
        } else {
            videoList.innerHTML = '<li>No videos found</li>';
        }
    } catch (error) {
        console.error("Error listing videos:", error);
        document.getElementById("videoList").innerHTML = '<li>Error loading videos</li>';
    }
}

// Initial load
document.addEventListener('DOMContentLoaded', async () => {
    await updateStatus();
    await listVideos();
    
    // Set up periodic updates
    setInterval(updateStatus, 1000);
    setInterval(listVideos, 5000);
}); 