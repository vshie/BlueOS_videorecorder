// Global variables
let isRecording = false;

async function updateStatus() {
    try {
        const response = await fetch('/status');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();
        const statusElement = document.getElementById("recordingStatus");
        const startButton = document.getElementById("startButton");
        const stopButton = document.getElementById("stopButton");
        
        isRecording = data.recording;
        
        if (isRecording) {
            statusElement.textContent = "Recording";
            statusElement.style.color = "red";
            startButton.disabled = true;
            stopButton.disabled = false;
        } else {
            statusElement.textContent = "Stopped";
            statusElement.style.color = "black";
            startButton.disabled = false;
            stopButton.disabled = true;
        }
    } catch (error) {
        console.error("Error updating status:", error);
    }
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
            await updateStatus();
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
            await updateStatus();
            await listVideos();
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
    const videoList = document.getElementById("videoList");
    try {
        const response = await fetch('/list');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
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
            videoList.innerHTML = '<li><em>No videos found</em></li>';
        }
    } catch (error) {
        console.error("Error listing videos:", error);
        videoList.innerHTML = '<li><em>Loading videos...</em></li>';
        // Retry once after a short delay
        setTimeout(listVideos, 1000);
    }
}

// Initial load
document.addEventListener('DOMContentLoaded', async () => {
    // Show loading state immediately
    document.getElementById("videoList").innerHTML = '<li><em>Loading videos...</em></li>';
    
    // Load initial state
    await Promise.all([
        updateStatus(),
        listVideos()
    ]);
    
    // Set up periodic updates with different intervals
    setInterval(updateStatus, 1000);  // Check status every second
    setInterval(listVideos, 5000);    // Update video list every 5 seconds
}); 