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

network DADCA_ACK_n20_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=149.50,148.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=221.56,178.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=265.15,133.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=286.48,225.14");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=341.90,148.64");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=341.44,341.39");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=383.95,411.69");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=331.02,480.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=289.43,424.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=232.32,454.05");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=190.59,389.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=239.17,349.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=149.78,342.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=188.85,271.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=114.21,234.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=249.11,278.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=362.31,209.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=440.50,144.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=481.14,217.10");
    }

}

