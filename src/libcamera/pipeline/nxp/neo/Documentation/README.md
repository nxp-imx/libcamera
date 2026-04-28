<!--
SPDX-License-Identifier: CC-BY-SA-4.0
-->

# Neo ISP pipeline handler documentation

This directory contains the Sphinx documentation for the Neo ISP pipeline handler.

## Prerequisites

- Python 3.7 or higher
- pip

## Setup and Build Instructions

### 1. Create a Virtual Environment

```bash
python3 -m venv venv
```

### 2. Activate the Virtual Environment

```bash
source venv/bin/activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Build the HTML Documentation

```bash
sphinx-build -b html source build/html
```

The generated HTML documentation will be available in the `build/html` directory.

### 5. View the Documentation

Open `build/html/index.html` in your web browser:

```bash
xdg-open build/html/index.html
```

## Deactivate Virtual Environment

When finished:

```bash
deactivate
```
