# NRSA Desktop App (Windows) — Setup Guide

এই ফাইলটি নতুন যোগ করা হয়েছে। আপনার আগের কোনো ফাইল (`src/`, `tests/`, `prototype/`,
`data/`, `docs/`, `CMakeLists.txt`, `README.md`, `LICENSE`) **একবিন্দুও পরিবর্তন করা
হয়নি** — শুধু নিচের নতুন ফাইল/ফোল্ডারগুলো যোগ করা হয়েছে যাতে পুরো প্রজেক্টটাকে একটা
রান-করা-যায় এমন Windows সফটওয়্যারে প্যাক করা যায়:

```
package.json          ← npm/electron-builder কনফিগ (এখান থেকেই npm.cmd কমান্ড চলবে)
.gitignore             ← node_modules / build আউটপুট বাদ দেওয়ার জন্য
electron/
  main.js              ← অ্যাপ চালু করার মূল কোড (স্প্ল্যাশ স্ক্রিন + মেইন উইন্ডো)
  preload.js           ← নিরাপদ, খালি preload script (ভবিষ্যতে C++ ইঞ্জিন যুক্ত করার জায়গা)
  splash.html          ← ETABS/SAP2000/AutoCAD-এর মতো লোগো-সহ লোডিং স্প্ল্যাশ স্ক্রিন
assets/
  logo.png             ← আপনার আপলোড করা লোগো (কালো ব্যাকগ্রাউন্ড সরিয়ে transparent করা)
  logo_master.png       ← হাই-রেজোলিউশন মূল লোগো
build/
  icon.ico             ← Windows .exe / টাস্কবার আইকন (16–256px, multi-resolution)
  icon.png             ← About ডায়ালগে ব্যবহৃত আইকন
DESKTOP_APP.md         ← এই ফাইলটি
```

## এটা আসলে কী করে (What this actually does)

`prototype/NRSA_RCC.html` আপনার প্রজেক্টের একমাত্র রান-করা-যায় এমন UI (একটা
সেলফ-কন্টেইনড HTML অ্যাপ)। Electron শুধু এই HTML ফাইলটাকে একটা নেটিভ Windows
উইন্ডোর ভেতরে চালায় — নিজের আইকন, টাইটেলবার, আর লোগো-সহ স্প্ল্যাশ স্ক্রিন দিয়ে —
ঠিক যেভাবে ETABS/SAP2000/AutoCAD ওপেন হওয়ার সময় লোগো দেখায়।

`src/`, `tests/` ফোল্ডারের C++ ইঞ্জিন (FEM সলভার, ডিজাইন মডিউল ইত্যাদি) এখনও শুধু
সোর্স কোড হিসেবেই আছে, এবং সেটা `CMakeLists.txt` দিয়ে আলাদাভাবে কম্পাইল করতে হয়
(Visual Studio / MSVC দিয়ে)। **npm দিয়ে C++ কোড কম্পাইল হয় না** — npm শুধু
Electron শেল (UI wrapper) বানায়। C++ ইঞ্জিনকে UI-এর সাথে সরাসরি যুক্ত করতে হলে
পরবর্তী ধাপে একটা bridge (যেমন local HTTP server বা native addon) লাগবে —
সেটা আলাদা কাজ, চাইলে পরে সেটাও করে দিতে পারি।

## প্রয়োজনীয় জিনিস (Prerequisites)

- Windows 10/11
- [Node.js LTS](https://nodejs.org) ইনস্টল করা থাকতে হবে (npm.cmd সহ আসে)
- ইন্টারনেট সংযোগ — প্রথমবার `npm.cmd install` চালানোর সময় Electron বাইনারি
  ডাউনলোড হবে, এবং অ্যাপ রান করার সময় Google Fonts + three.js + xlsx.js
  ইন্টারনেট থেকে লোড হয় (এগুলো `prototype/NRSA_RCC.html`-এর নিজস্ব রিসোর্স,
  এখানে পরিবর্তন করা হয়নি)

## আপনার চাওয়া মাত্র দুইটা কমান্ড

Windows-এর **cmd** (Command Prompt) খুলে এই ফোল্ডারে (যেখানে `package.json`
আছে) গিয়ে:

```cmd
npm.cmd install
npm.cmd run dist:win
```

- `npm.cmd install` → Electron ও electron-builder ডাউনলোড/ইনস্টল করবে
  (`node_modules/` তৈরি হবে, এটা `.gitignore`-এ বাদ দেওয়া আছে)
- `npm.cmd run dist:win` → Windows ইনস্টলার (.exe) বানাবে

আউটপুট পাবেন এখানে:

```
release/NRSA-Setup-0.1.0.exe
```

এই `.exe` ফাইলটা ডাবল-ক্লিক করলে ইনস্টলার চলবে, Desktop ও Start Menu শর্টকাট
তৈরি হবে, আর আপনার লোগোটাই হবে অ্যাপ আইকন। অ্যাপ চালু করলে প্রথমে লোগো-সহ
স্প্ল্যাশ স্ক্রিন দেখাবে, তারপর মূল উইন্ডো খুলবে।

দ্রুত টেস্ট করতে চাইলে ইনস্টলার না বানিয়ে সরাসরি চালাতে পারেন:

```cmd
npm.cmd start
```

## ⚠️ একটা গুরুত্বপূর্ণ নোট: `build` ফোল্ডারের নাম নিয়ে

`build/` ফোল্ডারটা এখন electron-builder-এর আইকন রাখার জায়গা (`build/icon.ico`)।
যদি আপনি C++ ইঞ্জিন (CMake) আলাদাভাবে কম্পাইল করেন, তখন out-of-source বিল্ড
ফোল্ডারের নাম `build` না দিয়ে অন্য কিছু দিন, যেমন:

```cmd
cmake -B out
cmake --build out --config Release
```

তাহলে দুই বিল্ড সিস্টেম একে অপরের সাথে সংঘর্ষ (collide) করবে না।

## GitHub-এ প্রকাশ করা (Publishing to GitHub)

`.gitignore` ইতিমধ্যে `node_modules/` আর `release/` বাদ দেওয়ার জন্য রেডি করা
আছে, তাই ভারী/জেনারেটেড ফাইল GitHub-এ যাবে না। ধাপগুলো:

```cmd
git init
git add .
git commit -m "Add Windows desktop packaging (Electron) with NRSA splash and icon"
git branch -M main
git remote add origin https://github.com/<your-username>/<your-repo>.git
git push -u origin main
```

(যদি প্রজেক্টটা আগে থেকেই একটা git repo হয়, শুধু শেষ তিন লাইন লাগবে —
নতুন করে `git init` করার দরকার নেই।)

## কী পরিবর্তন হয়নি (What was NOT changed)

`src/`, `tests/`, `examples/`, `data/`, `docs/`, `tools/`, `prototype/NRSA_RCC.html`,
`CMakeLists.txt`, `README.md`, `LICENSE` — এসব ফাইলের একটা লাইনও এডিট করা হয়নি।
শুধু উপরে তালিকাভুক্ত নতুন ফাইল/ফোল্ডার যোগ করা হয়েছে।
