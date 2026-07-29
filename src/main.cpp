#include "pch.h"

#include "BodyChain.h"
#include "Application.h"

#include "Foot.h"

int main()
{
	Application app(1600, 1200, "gecko");
	while (true) {
		app.render();
	}
	return 0;
}