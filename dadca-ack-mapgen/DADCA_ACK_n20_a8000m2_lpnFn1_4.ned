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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=36.08,50.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=25.50,116.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=116.16,27.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=164.70,125.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=247.27,144.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=341.41,152.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=338.54,240.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=275.90,293.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=272.53,386.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=182.10,378.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=153.34,439.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=68.84,412.42");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=92.74,350.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=22.85,335.34");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=7.34,492.69");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=94.89,503.45");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=52.58,555.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=9.21,597.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=96.12,607.89");
    }

}

