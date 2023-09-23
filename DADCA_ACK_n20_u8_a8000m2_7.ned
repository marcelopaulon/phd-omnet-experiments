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

network DADCA_ACK_n20_u8_a8000m2_7
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
        @display("i=misc/sensor;p=2292.22,7859.97");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2658.96,7809.81");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3321.62,7853.13");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3953.26,7896.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5095.62,7977.83");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=986.30,7571.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1797.10,7731.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3052.81,7618.40");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4928.56,7601.41");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5516.35,7543.95");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5948.27,7736.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=458.01,7468.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1448.84,7266.55");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2075.22,7296.56");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2498.71,7420.14");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3273.75,7290.99");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3638.90,7417.44");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4430.60,7384.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=125.55,7216.93");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1083.63,7101.53");
    }

}

