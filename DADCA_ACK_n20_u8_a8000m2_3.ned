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

network DADCA_ACK_n20_u8_a8000m2_3
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
        @display("i=misc/sensor;p=141.98,7815.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=592.02,7941.18");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1087.16,7828.58");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2523.50,7810.92");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4422.66,7948.10");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5831.40,7990.16");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1357.56,7512.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2038.31,7570.26");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2952.12,7652.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3555.14,7593.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4012.24,7596.60");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5265.05,7718.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=631.10,7321.09");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3236.81,7439.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4612.09,7496.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5892.61,7474.50");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=254.28,7157.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1028.59,7151.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1806.32,7219.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2194.09,7198.93");
    }

}

