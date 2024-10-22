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

network DADCA_ACK_n20_u3_a8000m2_8
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
        @display("i=misc/sensor;p=355.32,7967.94");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=875.26,7896.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2001.46,7866.59");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2758.51,7872.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3209.17,7919.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3604.87,7837.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4152.54,7902.13");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5186.72,7922.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1641.15,7632.32");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3315.25,7539.40");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3892.24,7542.06");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5450.01,7585.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=111.03,7431.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=611.94,7387.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1103.20,7475.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2378.42,7455.57");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2875.40,7420.18");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4433.46,7403.16");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4995.50,7290.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5860.48,7398.15");
    }

}

