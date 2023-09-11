package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n20_a8000m2_lpnFn2_0
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=156.95,136.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=194.08,81.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=250.95,120.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=314.65,110.82");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=303.95,188.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=289.45,268.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=233.43,208.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=413.28,263.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=462.34,329.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=85.40,21.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=99.73,183.87");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=245.30,46.51");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=313.81,48.02");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=379.65,88.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=401.00,189.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=309.47,356.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=168.06,263.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=488.97,257.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=402.56,341.11");
    }

}

