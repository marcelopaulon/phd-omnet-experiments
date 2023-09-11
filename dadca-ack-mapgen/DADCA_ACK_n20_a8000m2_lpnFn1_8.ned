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

network DADCA_ACK_n20_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=64.27,23.32");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=29.89,114.99");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=2.92,175.47");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=153.83,70.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=62.70,240.51");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=144.43,145.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=193.39,226.94");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=282.10,271.26");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=265.92,204.42");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=312.68,138.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=280.80,72.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=341.97,85.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=220.56,6.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=222.05,89.85");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=334.47,14.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=395.07,212.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=419.56,297.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=509.35,310.50");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=507.33,384.32");
    }

}

