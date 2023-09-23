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

network DADCA_ACK_n20_u4_a8000m2_3
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
        @display("i=misc/sensor;p=37.25,7936.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=567.12,7867.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1042.05,7852.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1683.47,7787.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2613.91,7990.04");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3280.15,7987.52");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3611.73,7794.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4868.74,7944.47");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5488.27,7965.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2139.29,7591.60");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3139.80,7598.59");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4494.21,7708.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4994.98,7610.08");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5625.53,7527.16");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=220.68,7380.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1504.67,7432.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2507.30,7430.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3710.42,7262.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4052.24,7498.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=840.66,7196.06");
    }

}

