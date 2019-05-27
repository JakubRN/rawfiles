import what3words
from os import environ
api_key = 'FRAZIWN6'
w3w = what3words.Geocoder(api_key)
res = w3w.convert_to_coordinates('prom.cape.pump')
print(res)
