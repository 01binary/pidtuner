import type { Metadata } from "next";
import { Inter } from "next/font/google";
import "./globals.css";

const inter = Inter({ subsets: ["latin"] });

export const metadata: Metadata = {
  title: "PID Tuner",
  description: "Motor controller tuner",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className={inter.className}>
        <script
          type="text/javascript"
          src="https://cdn.jsdelivr.net/npm/roslib@1/build/roslib.min.js"
          async
        />

        {children}
      </body>
    </html>
  );
}
