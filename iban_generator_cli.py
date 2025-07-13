#!/usr/bin/env python3
"""
Command-line Random IBAN Generator
Generate valid random International Bank Account Numbers (IBANs) for testing purposes.
"""

import random
import string
import sys
import argparse

# IBAN country configurations with their specific formats
IBAN_COUNTRIES = {
    'AD': {'length': 24, 'name': 'Andorra', 'bank_code_length': 4, 'branch_code_length': 4, 'account_length': 12},
    'AE': {'length': 23, 'name': 'United Arab Emirates', 'bank_code_length': 3, 'branch_code_length': 0, 'account_length': 16},
    'AL': {'length': 28, 'name': 'Albania', 'bank_code_length': 3, 'branch_code_length': 4, 'account_length': 16},
    'AT': {'length': 20, 'name': 'Austria', 'bank_code_length': 5, 'branch_code_length': 0, 'account_length': 11},
    'AZ': {'length': 28, 'name': 'Azerbaijan', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 20},
    'BA': {'length': 20, 'name': 'Bosnia and Herzegovina', 'bank_code_length': 3, 'branch_code_length': 3, 'account_length': 8},
    'BE': {'length': 16, 'name': 'Belgium', 'bank_code_length': 3, 'branch_code_length': 0, 'account_length': 9},
    'BG': {'length': 22, 'name': 'Bulgaria', 'bank_code_length': 4, 'branch_code_length': 4, 'account_length': 10},
    'BH': {'length': 22, 'name': 'Bahrain', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 14},
    'BR': {'length': 29, 'name': 'Brazil', 'bank_code_length': 8, 'branch_code_length': 5, 'account_length': 10},
    'BY': {'length': 28, 'name': 'Belarus', 'bank_code_length': 4, 'branch_code_length': 4, 'account_length': 16},
    'CH': {'length': 21, 'name': 'Switzerland', 'bank_code_length': 5, 'branch_code_length': 0, 'account_length': 12},
    'CR': {'length': 22, 'name': 'Costa Rica', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 14},
    'CY': {'length': 28, 'name': 'Cyprus', 'bank_code_length': 3, 'branch_code_length': 5, 'account_length': 16},
    'CZ': {'length': 24, 'name': 'Czech Republic', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 16},
    'DE': {'length': 22, 'name': 'Germany', 'bank_code_length': 8, 'branch_code_length': 0, 'account_length': 10},
    'DK': {'length': 18, 'name': 'Denmark', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 10},
    'DO': {'length': 28, 'name': 'Dominican Republic', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 20},
    'EE': {'length': 20, 'name': 'Estonia', 'bank_code_length': 2, 'branch_code_length': 0, 'account_length': 14},
    'EG': {'length': 29, 'name': 'Egypt', 'bank_code_length': 4, 'branch_code_length': 4, 'account_length': 17},
    'ES': {'length': 24, 'name': 'Spain', 'bank_code_length': 4, 'branch_code_length': 4, 'account_length': 10},
    'FI': {'length': 18, 'name': 'Finland', 'bank_code_length': 6, 'branch_code_length': 0, 'account_length': 8},
    'FO': {'length': 18, 'name': 'Faroe Islands', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 10},
    'FR': {'length': 27, 'name': 'France', 'bank_code_length': 5, 'branch_code_length': 5, 'account_length': 11},
    'GB': {'length': 22, 'name': 'United Kingdom', 'bank_code_length': 4, 'branch_code_length': 6, 'account_length': 8},
    'GE': {'length': 22, 'name': 'Georgia', 'bank_code_length': 2, 'branch_code_length': 0, 'account_length': 16},
    'GI': {'length': 23, 'name': 'Gibraltar', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 15},
    'GL': {'length': 18, 'name': 'Greenland', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 10},
    'GR': {'length': 27, 'name': 'Greece', 'bank_code_length': 3, 'branch_code_length': 4, 'account_length': 16},
    'GT': {'length': 28, 'name': 'Guatemala', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 20},
    'HR': {'length': 21, 'name': 'Croatia', 'bank_code_length': 7, 'branch_code_length': 0, 'account_length': 10},
    'HU': {'length': 28, 'name': 'Hungary', 'bank_code_length': 3, 'branch_code_length': 4, 'account_length': 16},
    'IE': {'length': 22, 'name': 'Ireland', 'bank_code_length': 4, 'branch_code_length': 6, 'account_length': 8},
    'IL': {'length': 23, 'name': 'Israel', 'bank_code_length': 3, 'branch_code_length': 3, 'account_length': 13},
    'IS': {'length': 26, 'name': 'Iceland', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 18},
    'IT': {'length': 27, 'name': 'Italy', 'bank_code_length': 5, 'branch_code_length': 5, 'account_length': 12},
    'JO': {'length': 30, 'name': 'Jordan', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 22},
    'KW': {'length': 30, 'name': 'Kuwait', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 22},
    'KZ': {'length': 20, 'name': 'Kazakhstan', 'bank_code_length': 3, 'branch_code_length': 0, 'account_length': 13},
    'LB': {'length': 28, 'name': 'Lebanon', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 20},
    'LC': {'length': 32, 'name': 'Saint Lucia', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 24},
    'LI': {'length': 21, 'name': 'Liechtenstein', 'bank_code_length': 5, 'branch_code_length': 0, 'account_length': 12},
    'LT': {'length': 20, 'name': 'Lithuania', 'bank_code_length': 5, 'branch_code_length': 0, 'account_length': 11},
    'LU': {'length': 20, 'name': 'Luxembourg', 'bank_code_length': 3, 'branch_code_length': 0, 'account_length': 13},
    'LV': {'length': 21, 'name': 'Latvia', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 13},
    'MC': {'length': 27, 'name': 'Monaco', 'bank_code_length': 5, 'branch_code_length': 5, 'account_length': 11},
    'MD': {'length': 24, 'name': 'Moldova', 'bank_code_length': 2, 'branch_code_length': 0, 'account_length': 18},
    'ME': {'length': 22, 'name': 'Montenegro', 'bank_code_length': 3, 'branch_code_length': 0, 'account_length': 15},
    'MK': {'length': 19, 'name': 'North Macedonia', 'bank_code_length': 3, 'branch_code_length': 0, 'account_length': 10},
    'MR': {'length': 27, 'name': 'Mauritania', 'bank_code_length': 5, 'branch_code_length': 5, 'account_length': 11},
    'MT': {'length': 31, 'name': 'Malta', 'bank_code_length': 4, 'branch_code_length': 5, 'account_length': 18},
    'MU': {'length': 30, 'name': 'Mauritius', 'bank_code_length': 4, 'branch_code_length': 2, 'account_length': 18},
    'NL': {'length': 18, 'name': 'Netherlands', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 10},
    'NO': {'length': 15, 'name': 'Norway', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 7},
    'PK': {'length': 24, 'name': 'Pakistan', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 16},
    'PL': {'length': 28, 'name': 'Poland', 'bank_code_length': 8, 'branch_code_length': 0, 'account_length': 16},
    'PS': {'length': 29, 'name': 'Palestine', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 21},
    'PT': {'length': 25, 'name': 'Portugal', 'bank_code_length': 4, 'branch_code_length': 4, 'account_length': 11},
    'QA': {'length': 29, 'name': 'Qatar', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 21},
    'RO': {'length': 24, 'name': 'Romania', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 16},
    'RS': {'length': 22, 'name': 'Serbia', 'bank_code_length': 3, 'branch_code_length': 0, 'account_length': 15},
    'SA': {'length': 24, 'name': 'Saudi Arabia', 'bank_code_length': 2, 'branch_code_length': 0, 'account_length': 18},
    'SE': {'length': 24, 'name': 'Sweden', 'bank_code_length': 3, 'branch_code_length': 0, 'account_length': 17},
    'SI': {'length': 19, 'name': 'Slovenia', 'bank_code_length': 5, 'branch_code_length': 0, 'account_length': 8},
    'SK': {'length': 24, 'name': 'Slovakia', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 16},
    'SM': {'length': 27, 'name': 'San Marino', 'bank_code_length': 5, 'branch_code_length': 5, 'account_length': 12},
    'TN': {'length': 24, 'name': 'Tunisia', 'bank_code_length': 2, 'branch_code_length': 3, 'account_length': 13},
    'TR': {'length': 26, 'name': 'Turkey', 'bank_code_length': 5, 'branch_code_length': 0, 'account_length': 17},
    'UA': {'length': 29, 'name': 'Ukraine', 'bank_code_length': 6, 'branch_code_length': 0, 'account_length': 19},
    'VG': {'length': 24, 'name': 'British Virgin Islands', 'bank_code_length': 4, 'branch_code_length': 0, 'account_length': 16},
    'XK': {'length': 20, 'name': 'Kosovo', 'bank_code_length': 2, 'branch_code_length': 2, 'account_length': 12},
}

def char_to_num(char):
    """Convert character to number for IBAN calculation (A=10, B=11, ..., Z=35)"""
    if char.isdigit():
        return char
    else:
        return str(ord(char.upper()) - ord('A') + 10)

def calculate_check_digits(country_code, bank_account_number):
    """Calculate IBAN check digits using MOD-97 algorithm"""
    # Create the rearranged string: bank_account_number + country_code + "00"
    rearranged = bank_account_number + country_code + "00"
    
    # Convert letters to numbers
    numeric_string = ""
    for char in rearranged:
        numeric_string += char_to_num(char)
    
    # Calculate MOD 97
    remainder = int(numeric_string) % 97
    
    # Check digits = 98 - remainder
    check_digits = 98 - remainder
    
    # Return as 2-digit string with leading zero if needed
    return f"{check_digits:02d}"

def generate_random_bank_code(length, alphanumeric=False):
    """Generate random bank code"""
    if alphanumeric:
        return ''.join(random.choices(string.ascii_uppercase + string.digits, k=length))
    else:
        return ''.join(random.choices(string.digits, k=length))

def generate_random_account_number(length, alphanumeric=False):
    """Generate random account number"""
    if alphanumeric:
        return ''.join(random.choices(string.ascii_uppercase + string.digits, k=length))
    else:
        return ''.join(random.choices(string.digits, k=length))

def generate_random_iban(country_code):
    """Generate a random valid IBAN for the specified country"""
    if country_code not in IBAN_COUNTRIES:
        raise ValueError(f"Country code {country_code} is not supported")
    
    config = IBAN_COUNTRIES[country_code]
    
    # Generate bank code
    bank_code = generate_random_bank_code(config['bank_code_length'])
    
    # Generate branch code if required
    branch_code = ""
    if config['branch_code_length'] > 0:
        branch_code = generate_random_bank_code(config['branch_code_length'])
    
    # Generate account number - some countries allow alphanumeric
    alphanumeric_countries = ['AD', 'AE', 'CY', 'FR', 'GB', 'GI', 'IT', 'LI', 'MC', 'MT', 'MU', 'SM']
    is_alphanumeric = country_code in alphanumeric_countries
    account_number = generate_random_account_number(config['account_length'], is_alphanumeric)
    
    # Combine bank code, branch code, and account number
    bank_account_number = bank_code + branch_code + account_number
    
    # Calculate check digits
    check_digits = calculate_check_digits(country_code, bank_account_number)
    
    # Create complete IBAN
    iban = country_code + check_digits + bank_account_number
    
    return iban

def format_iban(iban):
    """Format IBAN with spaces every 4 characters"""
    return ' '.join([iban[i:i+4] for i in range(0, len(iban), 4)])

def validate_iban(iban):
    """Validate an IBAN using the MOD-97 algorithm"""
    # Remove spaces and convert to uppercase
    iban = iban.replace(' ', '').upper()
    
    # Check if country code is supported
    country_code = iban[:2]
    if country_code not in IBAN_COUNTRIES:
        return False, f"Country code {country_code} is not supported"
    
    # Check length
    expected_length = IBAN_COUNTRIES[country_code]['length']
    if len(iban) != expected_length:
        return False, f"Invalid length. Expected {expected_length}, got {len(iban)}"
    
    # Rearrange: move first 4 characters to the end
    rearranged = iban[4:] + iban[:4]
    
    # Convert to numeric string
    numeric_string = ""
    for char in rearranged:
        numeric_string += char_to_num(char)
    
    # Calculate MOD 97
    remainder = int(numeric_string) % 97
    
    # Valid IBAN should have remainder 1
    if remainder == 1:
        return True, "Valid IBAN"
    else:
        return False, "Invalid IBAN check digits"

def list_countries():
    """List all supported countries"""
    print("Supported Countries:")
    print("===================")
    for code, info in sorted(IBAN_COUNTRIES.items()):
        print(f"{code} - {info['name']} (Length: {info['length']})")

def main():
    parser = argparse.ArgumentParser(
        description="Generate valid random International Bank Account Numbers (IBANs) for testing purposes",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  %(prog)s -c DE -n 5           # Generate 5 German IBANs
  %(prog)s -c US -n 10          # Generate 10 US IBANs
  %(prog)s -l                   # List all supported countries
  %(prog)s -v "DE89370400440532013000"  # Validate an IBAN
  %(prog)s -c FR -n 3 -f        # Generate 3 French IBANs with formatting
        """
    )
    
    parser.add_argument('-c', '--country', type=str, help='Country code (e.g., DE, US, FR)')
    parser.add_argument('-n', '--number', type=int, default=1, help='Number of IBANs to generate (default: 1)')
    parser.add_argument('-f', '--format', action='store_true', help='Format IBANs with spaces')
    parser.add_argument('-l', '--list', action='store_true', help='List all supported countries')
    parser.add_argument('-v', '--validate', type=str, help='Validate an IBAN')
    
    args = parser.parse_args()
    
    if args.list:
        list_countries()
        return
    
    if args.validate:
        is_valid, message = validate_iban(args.validate)
        if is_valid:
            print(f"✅ {message}")
        else:
            print(f"❌ {message}")
        return
    
    if not args.country:
        print("Error: Country code is required. Use -c/--country to specify.")
        print("Use -l/--list to see all supported countries.")
        sys.exit(1)
    
    country_code = args.country.upper()
    
    if country_code not in IBAN_COUNTRIES:
        print(f"Error: Country code '{country_code}' is not supported.")
        print("Use -l/--list to see all supported countries.")
        sys.exit(1)
    
    print(f"Generating {args.number} random IBAN(s) for {IBAN_COUNTRIES[country_code]['name']}:")
    print("=" * 60)
    
    for i in range(args.number):
        try:
            iban = generate_random_iban(country_code)
            if args.format:
                iban = format_iban(iban)
            print(iban)
        except Exception as e:
            print(f"Error generating IBAN: {e}")
            sys.exit(1)
    
    print("\n⚠️  Note: These are randomly generated IBANs for testing purposes only.")
    print("   They do not represent real bank accounts.")

if __name__ == "__main__":
    main()