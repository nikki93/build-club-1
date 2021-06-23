//
// WASM
//
var Module = { canvas: document.getElementById('canvas') }; // For Emscripten
(() => {
  // Fetch and launch core JS, trying to skip cache
  const s = document.createElement('script');
  s.async = true;
  s.type = 'text/javascript';
  s.src = '@PROJECT_NAME@.js?ts=' + new Date().getTime(); // CMake replaces filename
  document.getElementsByTagName('head')[0].appendChild(s);
})();

//
// Auto-reload
//
(() => {
  if (!window.location.href.includes('.xyz')) {
    const filenames = ['reload-trigger'];
    let lastUnfocusedTime = Date.now();
    let reloading = false;
    let interval;
    try {
      const getTimestampForFilename = async (filename) =>
        (await fetch(filename, { method: 'HEAD' })).headers.get('last-modified');
      const initialTimestamps = {};
      filenames.forEach(async (filename) => {
        initialTimestamps[filename] = await getTimestampForFilename(filename);
      });
      interval = setInterval(() => {
        const now = Date.now();
        if (!document.hasFocus() || now - lastUnfocusedTime < 3000) {
          filenames.forEach(async (filename) => {
            try {
              if (initialTimestamps[filename]) {
                const currentTimestamp = await getTimestampForFilename(filename);
                if (initialTimestamps[filename] != currentTimestamp) {
                  if (!reloading) {
                    reloading = true;
                    Module._JS_saveEditSession();
                    console.log('reloading...');
                    setTimeout(() => window.location.reload(), 60);
                  }
                }
              }
            } catch (e) {
              clearInterval(interval);
            }
          });
        }
        if (!document.hasFocus()) {
          lastUnfocusedTime = now;
        }
      }, 280);
    } catch (e) {
      clearInterval(interval);
    }
  }
})();

//
// UI
//
(() => {
  IncrementalDOM.attributes.value = IncrementalDOM.applyProp;

  window.UI = {};
  const UI = window.UI;

  UI.nextToken = 0;

  UI.isKeyboardCaptured = false;
  UI.eventCounts = new WeakMap();
  UI.setupEventListener = (target, type) => {
    target.addEventListener(type, (e) => {
      if (e.type === 'click' && e.detail === 0) {
        return;
      }
      const target = e.target;
      if (target.tagName === 'SUMMARY' && e.type === 'click') {
        e.preventDefault();
      }
      let counts = UI.eventCounts.get(target);
      if (counts === undefined) {
        counts = {};
        UI.eventCounts.set(target, counts);
        UI.noEvents = false;
      }
      const count = counts[e.type];
      if (count === undefined) {
        counts[e.type] = 1;
      } else {
        counts[e.type] = count + 1;
      }
    });
    if (target.tagName === 'INPUT') {
      target.addEventListener('focus', () => (UI.isKeyboardCaptured = true));
      target.addEventListener('blur', () => (UI.isKeyboardCaptured = false));
      target.addEventListener('keyup', (e) => {
        if (e.key === 'Enter') {
          target.blur();
        }
      });
    }
  };

  const oldPreventDefault = Event.prototype.preventDefault;
  Event.prototype.preventDefault = function() {
    if (!(this.target.tagName === 'INPUT' && this.key === 'Backspace')) {
      oldPreventDefault.bind(this)();
    }
  };

  const preventDefaultKeys = (e) => {
    if (!(e.target && e.target.tagName === 'INPUT') && e.code === 'Space') {
      e.preventDefault();
    }
    if (e.ctrlKey && (e.key === 'z' || e.key === 'y')) {
      e.preventDefault();
    }
  };
  window.addEventListener('keydown', preventDefaultKeys);
  window.addEventListener('keyup', preventDefaultKeys);
})();
