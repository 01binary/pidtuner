import { FC } from "react";
import styles from "./Invert.module.css";

type InvertProps = {
  onClick: () => void;
};

export const Invert: FC<InvertProps> = ({ onClick }) => (
  <div className={styles.invert}>
    <button onClick={onClick}>
      <svg
        width="98.4px"
        height="48px"
        viewBox="0 0 98.4 48"
      >
        <circle fill="#FFFFFF" stroke="#A5A5A5" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" cx="37.9" cy="33" r="6.7"/>
        <line fill="#FFFFFF" stroke="#000000" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="38" y1="36.3" x2="38" y2="29.7"/>
        <line fill="#FFFFFF" stroke="#000000" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="41.2" y1="33" x2="34.7" y2="33"/>
        <circle fill="#FFFFFF" stroke="#A5A5A5" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" cx="60.5" cy="15" r="6.7"/>
        <line fill="#FFFFFF" stroke="#000000" strokeLinecap="round" strokeLinejoin="round" strokeMiterlimit="10" x1="63.8" y1="15" x2="57.3" y2="15"/>
        <path fill="none" stroke="#424242" strokeLinecap="round" strokeLinejoin="round" d="M31.2,24c0-8.6,6.1-15.9,14.2-17.6"/>
        <polygon fill="#424242" points="52.5,6.3 42.7,10.8 44.9,6.6 42.3,2.6 				"/>
        <path fill="none" stroke="#424242" strokeLinecap="round" strokeLinejoin="round" d="M67.2,24c0,8.8-6.3,16.1-14.6,17.7"/>
        <polygon fill="#424242" points="45.6,41.6 55.4,37.4 53.2,41.5 55.6,45.5 				"/>
      </svg>
    </button>
  </div>
);
