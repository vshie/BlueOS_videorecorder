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
        console.error("Error updating status:", error);
    }
}

function updateRecordingDuration(startTime) {
    if (recordingTimer) {
        clearInterval(recordingTimer);
    }
    
    function updateDuration() {
        const now = new Date();
        const duration = Math.floor((now - startTime) / 1000); // Duration in seconds
        const hours = Math.floor(duration / 3600);
        const minutes = Math.floor((duration % 3600) / 60);
        const seconds = duration % 60;
        document.getElementById("recordingDuration").textContent = 
            `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
    }
    
    updateDuration(); // Update immediately
    recordingTimer = setInterval(updateDuration, 1000); // Then update every second
}

async function startRecording() {
    try {
        const splitDuration = document.getElementById("splitDuration").value;
        const response = await fetch(`/start?split_duration=${splitDuration}`);
        const data = await response.json();
        
        if (data.status === "success") {
            document.getElementById("errorMessage").textContent = "";
            document.getElementById("errorMessage").style.display = "none";
            updateStatus(); // Update status immediately after starting
        } else {
            document.getElementById("errorMessage").textContent = data.message || "Failed to start recording";
            document.getElementById("errorMessage").style.display = "block";
            document.getElementById("errorMessage").style.color = "red";
        }
    } catch (error) {
        console.error("Error starting recording:", error);
        document.getElementById("errorMessage").textContent = "Error starting recording";
        document.getElementById("errorMessage").style.display = "block";
        document.getElementById("errorMessage").style.color = "red";
    }
}

async function stopRecording() {
    try {
        const response = await fetch('/stop');
        const data = await response.json();
        
        if (data.status === "success") {
            document.getElementById("errorMessage").textContent = "";
            document.getElementById("errorMessage").style.display = "none";
            if (recordingTimer) {
                clearInterval(recordingTimer);
                recordingTimer = null;
            }
            document.getElementById("recordingDuration").textContent = "00:00:00";
            updateStatus(); // Update status immediately after stopping
        } else {
            document.getElementById("errorMessage").textContent = data.message || "Failed to stop recording";
            document.getElementById("errorMessage").style.display = "block";
            document.getElementById("errorMessage").style.color = "red";
        }
    } catch (error) {
        console.error("Error stopping recording:", error);
        document.getElementById("errorMessage").textContent = "Error stopping recording";
        document.getElementById("errorMessage").style.display = "block";
        document.getElementById("errorMessage").style.color = "red";
    }
}

async function listVideos() {
    try {
        const response = await fetch('/list');
        const data = await response.json();
        const videoList = document.getElementById("videoList");
        videoList.innerHTML = ''; // Clear existing list
        
        data.videos.forEach(video => {
            const li = document.createElement('li');
            const a = document.createElement('a');
            a.href = `/download/${video}`;
            a.textContent = video;
            li.appendChild(a);
            videoList.appendChild(li);
        });
        
        // Clear any error message if videos load successfully
        document.getElementById("errorMessage").textContent = "";
        document.getElementById("errorMessage").style.display = "none";
    } catch (error) {
        console.error("Error listing videos:", error);
        document.getElementById("errorMessage").textContent = "Error fetching videos";
        document.getElementById("errorMessage").style.display = "block";
        document.getElementById("errorMessage").style.color = "red";
    }
}

// Initial status and video list load
updateStatus();
listVideos();

// Refresh status and video list periodically
setInterval(updateStatus, 1000);
setInterval(listVideos, 5000); 