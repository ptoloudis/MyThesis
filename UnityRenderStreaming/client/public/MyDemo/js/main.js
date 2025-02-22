import { VideoPlayer } from "./video-player.js";
import { registerGamepadEvents, registerKeyboardEvents, registerMouseEvents, sendClickEvent } from "./register-events.js";
import { getServerConfig } from "../../js/config.js";

setup();

let playButton;
let videoPlayer;
let useWebSocket;

window.document.oncontextmenu = function () {
  return false;     // cancel default menu
};

window.addEventListener('resize', function () {
  videoPlayer.resizeVideo();
}, true);

window.addEventListener('beforeunload', async () => {
  await videoPlayer.stop();
}, true);

async function setup() {
  const res = await getServerConfig();
  useWebSocket = res.useWebSocket;
  showWarningIfNeeded(res.startupMode);
  showPlayButton();
}

function showWarningIfNeeded(startupMode) {
  const warningDiv = document.getElementById("warning");
  if (startupMode == "private") {
    warningDiv.innerHTML = "<h4>Warning</h4> This sample is not working on Private Mode.";
    warningDiv.hidden = false;
  }
}

function showPlayButton() {
  if (!document.getElementById('playButton')) {
    let elementPlayButton = document.createElement('img');
    elementPlayButton.id = 'playButton';
    elementPlayButton.src = 'images/Play.png';
    elementPlayButton.alt = 'Start Streaming';
    playButton = document.getElementById('player').appendChild(elementPlayButton);
    playButton.addEventListener('click', onClickPlayButton);
  }
}

function onClickPlayButton() {

  playButton.style.display = 'none';

  const playerDiv = document.getElementById('player');

  // add first video player
  const videoContainer1 = document.createElement('div');
  videoContainer1.className = 'video-container';
  const whiteBox1 = document.createElement('div');
  whiteBox1.style.border = '5px solid white';
  whiteBox1.style.padding = '10px';
  const elementVideo1 = document.createElement('video');
  elementVideo1.id = 'Video1';
  elementVideo1.name = 'Drone';
  elementVideo1.style.touchAction = 'none';
  whiteBox1.appendChild(elementVideo1);
  const nameDisplay1 = document.createElement('div');
  nameDisplay1.textContent = elementVideo1.name;
  nameDisplay1.className = 'video-name';
  whiteBox1.appendChild(nameDisplay1);
  videoContainer1.appendChild(whiteBox1);
  playerDiv.appendChild(videoContainer1);

  // add second video player
  const videoContainer2 = document.createElement('div');
  videoContainer2.className = 'video-container';
  const whiteBox2 = document.createElement('div');
  whiteBox2.style.border = '5px solid white';
  whiteBox2.style.padding = '10px';
  const elementVideo2 = document.createElement('video');
  elementVideo2.id = 'Video2';
  elementVideo2.name = 'Rover';
  elementVideo2.style.touchAction = 'none';
  whiteBox2.appendChild(elementVideo2);
  const nameDisplay2 = document.createElement('div');
  nameDisplay2.textContent = elementVideo2.name;
  nameDisplay2.className = 'video-name';
  whiteBox2.appendChild(nameDisplay2);
  videoContainer2.appendChild(whiteBox2);
  playerDiv.appendChild(videoContainer2);

  setupVideoPlayer([elementVideo1, elementVideo2]).then(value => videoPlayer = value);

  // add fullscreen button
  const elementFullscreenButton = document.createElement('img');
  elementFullscreenButton.id = 'fullscreenButton';
  elementFullscreenButton.src = 'images/FullScreen.png';
  playerDiv.appendChild(elementFullscreenButton);
  
  elementFullscreenButton.addEventListener("click", function () {
    if (!document.fullscreenElement || !document.webkitFullscreenElement) {
      if (document.documentElement.requestFullscreen) {
        document.documentElement.requestFullscreen();
      }
      else if (document.documentElement.webkitRequestFullscreen) {
        document.documentElement.webkitRequestFullscreen(Element.ALLOW_KEYBOARD_INPUT);
      } else {
        if (playerDiv.style.position == "absolute") {
          playerDiv.style.position = "relative";
        } else {
          playerDiv.style.position = "absolute";
        }
      }
    }
  });
  document.addEventListener('webkitfullscreenchange', onFullscreenChange);
  document.addEventListener('fullscreenchange', onFullscreenChange);

  function onFullscreenChange() {
    if (document.webkitFullscreenElement || document.fullscreenElement) {
      playerDiv.style.position = "absolute";
      elementFullscreenButton.style.display = 'none';
    }
    else {
      playerDiv.style.position = "relative";
      elementFullscreenButton.style.display = 'block';
    }
  }
}

async function setupVideoPlayer(elements) {
  const videoPlayer = new VideoPlayer(elements);
  await videoPlayer.setupConnection(useWebSocket);

  videoPlayer.ondisconnect = onDisconnect;
  registerGamepadEvents(videoPlayer);
  registerKeyboardEvents(videoPlayer);
  registerMouseEvents(videoPlayer, elements[0]);

  return videoPlayer;
}

function onDisconnect() {
  const playerDiv = document.getElementById('player');
  clearChildren(playerDiv);
  videoPlayer.stop();
  videoPlayer = null;
  showPlayButton();
}

function clearChildren(element) {
  while (element.firstChild) {
    element.removeChild(element.firstChild);
  }
}
