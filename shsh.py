angles = [150, 301, 28, 5]  # Sostituisci con i tuoi valori

# Verifica che tutti gli angoli siano compresi tra 0 e 300
are_valid = all(0 <= angle <= 300 for angle in angles)

print("Tutti gli angoli sono validi:", are_valid)