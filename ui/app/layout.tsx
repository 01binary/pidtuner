import type { Metadata } from "next";
import { Inter } from "next/font/google";
import { Plot } from "./components/Plot";
import { Navigation } from "./components/Navigation";
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
          <Plot />
        </header>

        <nav>
          <Navigation />
        </nav>

        <main>
          {children}
        </main>
      </body>
    </html>
  );
}
