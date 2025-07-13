# 🏦 Random IBAN Generator

A comprehensive tool to generate valid random International Bank Account Numbers (IBANs) for testing purposes. This tool supports 75+ countries and includes both a command-line interface and a web application.

## ⚠️ Important Notice

**These are randomly generated IBANs for testing purposes only. They do not represent real bank accounts.**

## Features

- ✅ **Valid IBAN Generation**: Uses proper MOD-97 algorithm for check digit calculation
- 🌍 **75+ Countries Supported**: Comprehensive support for international IBAN formats
- 🎯 **Country-Specific Formatting**: Follows each country's specific IBAN structure
- 📝 **Alphanumeric Support**: Handles countries that use letters in their IBANs
- 🔍 **IBAN Validation**: Built-in validator to check IBAN format and check digits
- 💻 **Dual Interface**: Both command-line and web-based tools
- 📄 **Export Functionality**: Download generated IBANs as text files
- 🎨 **User-Friendly**: Clean, intuitive interface with helpful information

## Installation

### Prerequisites

- Python 3.7 or higher
- pip (Python package installer)

### Quick Setup

1. **Clone or download** this repository
2. **Install dependencies**:
   ```bash
   pip install streamlit
   ```

### Using Virtual Environment (Recommended)

```bash
# Create virtual environment
python3 -m venv iban_env

# Activate virtual environment
source iban_env/bin/activate  # On Linux/Mac
# or
iban_env\Scripts\activate     # On Windows

# Install dependencies
pip install streamlit
```

## Usage

### 🌐 Web Application (Recommended)

Launch the interactive web interface:

```bash
streamlit run random_iban_generator.py
```

This will open a browser window with the full-featured IBAN generator including:
- Country selection dropdown
- Number of IBANs to generate
- Real-time IBAN validation
- Download functionality
- Educational information about IBANs

### 💻 Command Line Interface

For quick generation and automation:

```bash
python iban_generator_cli.py [OPTIONS]
```

#### CLI Options

- `-c, --country`: Country code (e.g., DE, US, FR)
- `-n, --number`: Number of IBANs to generate (default: 1)
- `-f, --format`: Format IBANs with spaces
- `-l, --list`: List all supported countries
- `-v, --validate`: Validate an IBAN

#### CLI Examples

```bash
# Generate 5 German IBANs
python iban_generator_cli.py -c DE -n 5

# Generate 10 formatted French IBANs
python iban_generator_cli.py -c FR -n 10 -f

# List all supported countries
python iban_generator_cli.py -l

# Validate an IBAN
python iban_generator_cli.py -v "DE89370400440532013000"

# Generate single UK IBAN
python iban_generator_cli.py -c GB
```

## Supported Countries

The tool supports 75+ countries including:

| Code | Country | Length | Code | Country | Length |
|------|---------|--------|------|---------|--------|
| AD | Andorra | 24 | AE | United Arab Emirates | 23 |
| AL | Albania | 28 | AT | Austria | 20 |
| AZ | Azerbaijan | 28 | BA | Bosnia and Herzegovina | 20 |
| BE | Belgium | 16 | BG | Bulgaria | 22 |
| BH | Bahrain | 22 | BR | Brazil | 29 |
| BY | Belarus | 28 | CH | Switzerland | 21 |
| CR | Costa Rica | 22 | CY | Cyprus | 28 |
| CZ | Czech Republic | 24 | DE | Germany | 22 |
| DK | Denmark | 18 | DO | Dominican Republic | 28 |
| EE | Estonia | 20 | EG | Egypt | 29 |
| ES | Spain | 24 | FI | Finland | 18 |
| FO | Faroe Islands | 18 | FR | France | 27 |
| GB | United Kingdom | 22 | GE | Georgia | 22 |
| GI | Gibraltar | 23 | GL | Greenland | 18 |
| GR | Greece | 27 | GT | Guatemala | 28 |
| HR | Croatia | 21 | HU | Hungary | 28 |
| IE | Ireland | 22 | IL | Israel | 23 |
| IS | Iceland | 26 | IT | Italy | 27 |
| JO | Jordan | 30 | KW | Kuwait | 30 |
| KZ | Kazakhstan | 20 | LB | Lebanon | 28 |
| LC | Saint Lucia | 32 | LI | Liechtenstein | 21 |
| LT | Lithuania | 20 | LU | Luxembourg | 20 |
| LV | Latvia | 21 | MC | Monaco | 27 |
| MD | Moldova | 24 | ME | Montenegro | 22 |
| MK | North Macedonia | 19 | MR | Mauritania | 27 |
| MT | Malta | 31 | MU | Mauritius | 30 |
| NL | Netherlands | 18 | NO | Norway | 15 |
| PK | Pakistan | 24 | PL | Poland | 28 |
| PS | Palestine | 29 | PT | Portugal | 25 |
| QA | Qatar | 29 | RO | Romania | 24 |
| RS | Serbia | 22 | SA | Saudi Arabia | 24 |
| SE | Sweden | 24 | SI | Slovenia | 19 |
| SK | Slovakia | 24 | SM | San Marino | 27 |
| TN | Tunisia | 24 | TR | Turkey | 26 |
| UA | Ukraine | 29 | VG | British Virgin Islands | 24 |
| XK | Kosovo | 20 | | | |

## IBAN Format Overview

An IBAN (International Bank Account Number) consists of:

1. **Country Code**: 2 letters (ISO 3166-1 alpha-2)
2. **Check Digits**: 2 digits (calculated using MOD-97 algorithm)
3. **Basic Bank Account Number (BBAN)**: Up to 30 alphanumeric characters
   - Bank code
   - Branch code (if applicable)
   - Account number

### Example IBAN Structure

```
DE89 3704 0044 0532 0130 00
│└─┘ └──────────────────────┘
│ │           │
│ │           └── Basic Bank Account Number (BBAN)
│ └── Check Digits
└── Country Code
```

## Validation

The tool implements proper IBAN validation using:

- **Length Validation**: Ensures correct length for each country
- **Character Validation**: Checks for valid characters in each position
- **Check Digit Validation**: Uses MOD-97 algorithm as per ISO 13616
- **Country-Specific Rules**: Follows each country's specific format requirements

## Technical Details

### Check Digit Calculation

The tool uses the standard MOD-97 algorithm:

1. Move the first 4 characters to the end
2. Replace letters with numbers (A=10, B=11, ..., Z=35)
3. Calculate mod 97 of the resulting number
4. Valid IBAN should have remainder 1

### Country-Specific Features

- **Alphanumeric Support**: Some countries (FR, GB, IT, etc.) allow letters
- **Variable Structure**: Different bank code and account number lengths
- **Proper Formatting**: Country-specific spacing and presentation

## Files

- `random_iban_generator.py`: Streamlit web application
- `iban_generator_cli.py`: Command-line interface
- `README.md`: This documentation file

## Development

### Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

### Adding New Countries

To add support for a new country:

1. Add country configuration to `IBAN_COUNTRIES` dictionary
2. Include: country code, name, length, bank code length, branch code length, account length
3. Add to alphanumeric countries list if applicable
4. Test with known valid IBANs from that country

## License

This project is open source and available under the MIT License.

## Disclaimer

This tool is designed for testing and development purposes only. The generated IBANs:

- Are mathematically valid according to ISO 13616
- Do not represent real bank accounts
- Should not be used for any fraudulent activities
- Are generated randomly and may coincidentally match real account numbers

Always use test data responsibly and in accordance with your local laws and regulations.

## Support

For issues, questions, or contributions:

1. Check the documentation above
2. Review existing issues
3. Create a new issue with detailed information
4. Include sample inputs and expected outputs

---

**Happy Testing! 🚀**