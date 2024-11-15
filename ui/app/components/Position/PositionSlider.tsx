import { inter } from "../../inter";

export const PositionSlider = () => {
  return (
    <svg
      width="200px"
      height="96px"
      viewBox="0 0 200 96"
    >
      <rect
        id="error"
        x="17.7"
        y="17"
        fill="#D3D3D3"
        width="25.9"
        height="29.6"
      />

      <rect
        id="track"
        x="17.7"
        y="17"
        fill="none"
        stroke="#A5A5A5"
        width="163"
        height="29.5"
      />

      <g id="scale">
        <line stroke="#A5A5A5" x1="99.1" y1="57.3" x2="99.1" y2="72.1"/>
        <line stroke="#A5A5A5" x1="139.9" y1="57.3" x2="139.9" y2="69.3"/>
        <line stroke="#A5A5A5" x1="160.3" y1="57.3" x2="160.3" y2="64.7"/>
        <line stroke="#A5A5A5" x1="119.5" y1="57.3" x2="119.5" y2="64.7"/>
        <line stroke="#A5A5A5" x1="58.3" y1="57.3" x2="58.3" y2="69.3"/>
        <line stroke="#A5A5A5" x1="78.7" y1="57.3" x2="78.7" y2="64.7"/>
        <line stroke="#A5A5A5" x1="38" y1="57.3" x2="38" y2="64.7"/>

        <polyline
          fill="none"
          stroke="#A5A5A5"
          points="180.7,72.1 180.7,57.3 180.5,57.3 17.6,57.3 17.6,72.1 	"
        />

        <text
          transform="matrix(1 0 0 1 12.4579 93.6534)"
          fontFamily={inter.style.fontFamily}
          fontSize="18px"
        >
          0
        </text>

        <text
          transform="matrix(1 0 0 1 165.4023 93.6534)"
          fontFamily={inter.style.fontFamily}
          fontSize="18px"
        >
          100
        </text>
      </g>

      <line
        id="current"
        stroke="#424242"
        strokeWidth="6"
        strokeLinejoin="round"
        x1="17.7"
        y1="46.7"
        x2="43.6"
        y2="46.7"
      />
  
      <polygon
        id="head"
        fill="#FFFFFF"
        stroke="#000000"
        strokeLinecap="round"
        strokeLinejoin="round"
        points="9.5,16.9 17.7,29.8 26,16.9 26,5.3 9.5,5.3"
      />
    </svg>
  )
};
