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

network DADCA_ACK_n20_u2_a8000m2_8
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
        @display("i=misc/sensor;p=725.48,7763.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1199.80,7933.01");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1881.64,7832.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2229.43,7785.14");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3287.71,7830.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4858.54,7828.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=213.45,7750.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2658.15,7691.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3682.75,7741.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4350.11,7591.71");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=480.36,7354.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=875.28,7303.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1311.66,7386.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1740.54,7302.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2387.08,7271.31");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3084.18,7349.25");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4655.24,7415.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5208.92,7398.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=86.19,7225.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3532.67,7162.01");
    }

}

