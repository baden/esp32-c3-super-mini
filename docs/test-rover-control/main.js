import './style.css'
import javascriptLogo from './javascript.svg'
import viteLogo from '/vite.svg'
import { setupCounter } from './counter.js'

document.querySelector('#app').innerHTML = `
  <div>
    <a href="https://vitejs.dev" target="_blank">
      <img src="${viteLogo}" class="logo" alt="Vite logo" />
    </a>
    <a href="https://developer.mozilla.org/en-US/docs/Web/JavaScript" target="_blank">
      <img src="${javascriptLogo}" class="logo vanilla" alt="JavaScript logo" />
    </a>
    <h1>Hello Vite!</h1>
    <div class="card">
      <canvas id="L" width="200" height="200"></canvas>
    </div>
    <button id="generateL">Generate L</button>
    <p class="read-the-docs">
      Click on the Vite logo to learn more
    </p>
  </div>
`

const canvas = document.querySelector('#L');
const ctx = canvas.getContext('2d');
const image = ctx.createImageData(200, 200);

document.querySelector('#generateL').addEventListener('click', () => {
  // const image = ctx.get
  for (let y = 0; y < 200; y++) {
    for (let x = 0; x < 200; x++) {
      const index = (x + y * 200) * 4;

      const x_stick = (x - 100)/100;
      const y_stick = (100 - y)/100;

      let k1 = x_stick * y_stick;
      let k2 = x_stick * x_stick * y_stick * y_stick;
      let left_wheel = (x_stick + y_stick) * (1 * (1-k1) + 0.5 * k1);
      let right_wheel = y_stick - x_stick;

      image.data[index + 0] = (left_wheel * 255 * 1) % 255;
      image.data[index + 1] = left_wheel * 255;
      image.data[index + 2] = 0;
      image.data[index + 3] = 255;
    }
  }
  ctx.putImageData(image, 0, 0);
});


// setupCounter(document.querySelector('#counter'))
