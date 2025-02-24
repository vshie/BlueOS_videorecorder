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
        const errorMsg = document.getElementById("errorMessage");
        
        isRecording = data.recording;
        
        if (isRecording) {
            statusElement.textContent = "Recording";
            statusElement.style.color = "red";
            startButton.disabled = true;
            stopButton.disabled = false;
            errorMsg.style.display = "none";
        } else {
            statusElement.textContent = "Stopped";
            statusElement.style.color = "black";
            startButton.disabled = false;
            stopButton.disabled = true;
        }
    } catch (error) {
        console.error("Error updating status:", error);
        document.getElementById("errorMessage").textContent = "Error updating status: " + error.message;
        document.getElementById("errorMessage").style.display = "block";
    }
}

async function startRecording() {
    const errorMsg = document.getElementById("errorMessage");
    const startButton = document.getElementById("startButton");
    const stopButton = document.getElementById("stopButton");
    
    try {
        startButton.disabled = true; // Disable immediately to prevent double-clicks
        const response = await fetch('/start', {
            method: 'GET'
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.message || `HTTP error! status: ${response.status}`);
        }
        
        if (data.success) {
            errorMsg.style.display = "none";
            stopButton.disabled = false;
        } else {
            throw new Error(data.message || "Failed to start recording");
        }
    } catch (error) {
        console.error("Error starting recording:", error);
        errorMsg.textContent = "Error starting recording: " + error.message;
        errorMsg.style.display = "block";
        startButton.disabled = false;
        stopButton.disabled = true;
    }
    
    await updateStatus();
}

async function stopRecording() {
    const errorMsg = document.getElementById("errorMessage");
    const stopButton = document.getElementById("stopButton");
    
    try {
        stopButton.disabled = true; // Disable immediately to prevent double-clicks
        const response = await fetch('/stop', {
            method: 'GET'
        });
        
        const data = await response.json();
        
        if (!response.ok) {
            throw new Error(data.message || `HTTP error! status: ${response.status}`);
        }
        
        if (data.success) {
            errorMsg.style.display = "none";
            await listVideos();
        } else {
            throw new Error(data.message || "Failed to stop recording");
        }
    } catch (error) {
        console.error("Error stopping recording:", error);
        errorMsg.textContent = "Error stopping recording: " + error.message;
        errorMsg.style.display = "block";
    }
    
    await updateStatus();
}

async function listVideos() {
    const videoList = document.getElementById("videoList");
    const errorMsg = document.getElementById("errorMessage");
    
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
            errorMsg.style.display = "none";
        } else {
            videoList.innerHTML = '<li><em>No videos found</em></li>';
        }
    } catch (error) {
        console.error("Error listing videos:", error);
        videoList.innerHTML = '<li><em>Error loading videos</em></li>';
        errorMsg.textContent = "Error loading videos: " + error.message;
        errorMsg.style.display = "block";
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