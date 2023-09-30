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

network DADCA_ACK_n20_u4_a8000m2_9
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
        @display("i=misc/sensor;p=1239.02,7965.14");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1866.09,7967.68");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2716.57,7909.77");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3543.40,7901.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4046.97,7986.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4424.96,7976.06");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4857.19,7858.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=176.91,7565.25");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=940.07,7645.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1533.55,7577.53");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2397.95,7727.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4034.10,7553.29");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4506.58,7505.97");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5411.32,7713.11");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=640.72,7438.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3036.50,7471.36");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3534.74,7468.39");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5154.02,7454.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=124.95,7126.61");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=984.23,7160.02");
    }

}

