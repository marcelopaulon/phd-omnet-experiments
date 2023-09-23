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

network DADCA_ACK_n20_u8_a8000m2_6
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
        @display("i=misc/sensor;p=3972.42,7953.06");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4396.26,7974.83");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4747.20,7968.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5489.08,7819.86");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5944.04,7943.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2043.26,7694.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2508.82,7717.68");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3020.90,7693.95");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3637.40,7624.24");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5195.22,7574.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1878.31,7382.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3922.93,7343.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4290.39,7314.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4837.48,7266.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5538.69,7464.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5915.90,7503.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2561.96,7242.26");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2940.99,7052.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3349.11,7145.23");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5191.97,7032.53");
    }

}

