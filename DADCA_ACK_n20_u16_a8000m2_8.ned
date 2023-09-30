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

network DADCA_ACK_n20_u16_a8000m2_8
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
        @display("i=misc/sensor;p=188.01,7994.05");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=765.75,7959.64");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1253.44,7828.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1800.01,7986.67");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2330.98,7771.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3048.09,7918.90");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3669.92,7995.03");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5597.45,7921.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=32.03,7672.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=530.25,7649.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2623.64,7544.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3465.45,7591.86");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4114.12,7592.62");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4453.46,7695.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4995.04,7670.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5844.57,7627.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=316.83,7332.34");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=964.41,7431.88");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1601.76,7419.33");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2058.11,7402.99");
    }

}

