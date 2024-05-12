import re

def separate_data(output_dir):
    with open("/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal/output.txt", "r") as file:
        content = file.read()

    # Extraction des données non filtrées et filtrées à l'aide des expressions régulières
    pattern = r"NON FILTREE\s+(.*?)\s+FILTREE\s+(.*)"
    match = re.search(pattern, content, re.DOTALL)
    if match:
        non_filtree_content = match.group(1).strip()
        filtree_content = match.group(2).strip()

        # Écriture des entiers non filtrés dans NonFiltre.txt
        non_filtree_numbers = non_filtree_content.split()
        with open(f"{output_dir}/NonFiltre.txt", "w") as nf_file:
            nf_file.write('\n'.join(non_filtree_numbers))
    
        # Écriture des entiers filtrés dans Filtre.txt
        filtree_numbers = filtree_content.split()
        with open(f"{output_dir}/Filtre.txt", "w") as f_file:
            f_file.write('\n'.join(filtree_numbers))

        # Trouver la ligne du temps de filtrage moyen
        time_pattern = r"Temps de filtrage moyen par échantillon: (\d+) microsecondes"
        time_match = re.search(time_pattern, content)
        if time_match:
            avg_time = time_match.group(0)  # capture de toute la ligne

            # Écriture du récapitulatif des compteurs et du temps moyen de filtrage
            with open(f"{output_dir}/recap.txt", "w") as recap_file:
                recap_file.write(f"Non Filtre: {len(non_filtree_numbers)} entiers\n")
                recap_file.write(f"Filtre: {len(filtree_numbers)} entiers\n")
                recap_file.write(avg_time)  # écriture de la ligne du temps moyen de filtrage

# Appel de la fonction pour séparer les données et créer le récapitulatif pour le dossier spécifié
separate_data("/Users/mathiasrechsteiner/Desktop/Cours ING 3/Semestre 2/Traitement du Signal")
separate_data("/Users/mathiasrechsteiner/Documents/Processing/Test_Project_DUE")
