#include "pch.h"

#include "BodyChain.h"
#include "Application.h"

#include "Foot.h"

int main()
{
	Application app("gecko");
	while (true) {
		app.render();
		app.update();
	}
	return 0;
}