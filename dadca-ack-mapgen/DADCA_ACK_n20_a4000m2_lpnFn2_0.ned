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

network DADCA_ACK_n20_a4000m2_lpnFn2_0
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
        @display("i=misc/sensor,p=44.69,156.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=126.86,192.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=6.67,88.91");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=184.59,109.08");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=196.36,182.83");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=248.91,229.05");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=174.95,264.67");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=234.21,328.44");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=254.14,61.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=157.45,34.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=10.79,220.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=72.55,253.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=52.64,28.94");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=243.19,136.02");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=288.57,178.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=335.18,238.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=120.12,314.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=307.65,339.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=318.13,116.12");
    }

}

