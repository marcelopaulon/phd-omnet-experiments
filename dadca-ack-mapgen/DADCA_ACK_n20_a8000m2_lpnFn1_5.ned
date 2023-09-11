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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=132.45,45.85");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=46.35,17.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=28.55,98.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=158.78,114.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=171.98,210.23");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=177.01,306.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=92.33,380.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=35.59,351.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=279.42,351.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=337.83,388.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=268.41,432.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=197.35,434.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=219.51,362.21");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=395.43,418.16");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=459.53,380.07");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=488.95,451.63");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=519.30,508.25");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=514.50,588.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=582.16,515.14");
    }

}

