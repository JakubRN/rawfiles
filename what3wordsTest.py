import what3words
from os import environ
api_key = 'FRAZIWN6'
w3w = what3words.Geocoder(api_key)
res = w3w.convert_to_coordinates('balanced.riches.soup')
print(res)
