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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=159.78,150.94");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=190.73,245.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=92.18,238.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=129.88,285.45");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=82.72,330.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=59.20,387.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=23.97,308.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=61.49,158.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=222.77,113.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=210.08,51.32");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=153.57,30.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=37.16,61.57");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=294.60,135.75");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=239.72,203.96");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=375.26,77.61");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=440.70,150.73");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=489.29,236.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=483.21,80.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=366.89,17.63");
    }

}

