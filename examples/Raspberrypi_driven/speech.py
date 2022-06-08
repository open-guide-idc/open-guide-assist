
import speech_recognition as sr

r = sr.Recognizer()
print(sr.Microphone.list_microphone_names())
with sr.Microphone(device_index=6) as source:
    print("You have entered the scanning mode:")
    audio=r.adjust_for_ambient_noise(source)
    audio=r.listen(source)
    
    try:
        text=r.recognize_google(audio)
        print("You said: " + text)
    except sr.UnknownValueError:
        print('Sorry could not recognize voice')
    except sr.RequestError as e:
        print("error 2")
        
