import type { Metadata } from "next";
import { Inter } from "next/font/google";
import { Plot } from "./components/Plot";
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
        <header>
          <h1>Motor Control</h1>
          <Plot />
        </header>

        <main>
          {children}
        </main>
      </body>
    </html>
  );
}
