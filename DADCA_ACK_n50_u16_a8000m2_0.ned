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

network DADCA_ACK_n50_u16_a8000m2_0
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
        @display("i=misc/sensor;p=548.98,7918.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1146.32,7779.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1903.85,7864.39");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2539.67,7995.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3288.85,7876.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3680.67,7994.80");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4029.28,7763.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4362.67,7953.85");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4957.58,7964.87");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=37.20,7568.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1459.37,7599.36");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2858.37,7550.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3368.11,7511.77");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4543.67,7540.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5272.91,7706.10");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1105.63,7373.37");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2056.80,7431.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3837.00,7356.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=626.02,7166.90");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1721.19,7198.20");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2870.05,7066.57");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3272.72,7109.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4844.81,7211.39");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5729.74,7054.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1293.50,6910.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2225.31,6898.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3594.80,6768.38");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4178.50,6905.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4533.28,6996.82");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4819.64,6778.85");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5154.80,6922.40");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=187.55,6701.01");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=566.95,6528.69");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=920.78,6755.30");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1665.53,6613.09");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2299.22,6553.22");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5482.63,6728.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1101.66,6305.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2837.22,6405.95");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3819.91,6278.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4143.06,6504.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5345.24,6380.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=819.02,6021.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1945.35,6176.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2625.14,6097.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=339.50,5873.73");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1566.06,5799.31");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2261.88,5933.97");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3133.22,5831.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3878.12,5838.08");
    }

}

