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

network DADCA_ACK_n100_u3_a8000m2_2
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
        @display("i=misc/sensor;p=1778.27,7951.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2942.52,7979.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4061.74,7840.47");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4419.04,7784.22");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4805.76,7993.95");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5171.23,7965.56");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=32.38,7563.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=960.71,7540.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1298.37,7644.14");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2510.43,7510.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3084.47,7589.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3438.41,7608.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4958.71,7626.39");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5588.63,7588.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5974.18,7624.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1859.19,7489.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2832.68,7328.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5368.70,7311.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=210.69,7129.59");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1358.92,7216.87");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2084.58,7162.99");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3792.24,7182.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4243.41,7254.69");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4690.00,7187.07");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5950.96,7245.21");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=517.97,6900.28");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1651.53,7002.51");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2365.69,6836.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2836.98,6933.20");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3303.37,6971.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3664.63,6773.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5094.82,6871.76");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5583.36,6905.08");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=152.68,6603.25");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=613.19,6518.21");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1079.43,6691.13");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4191.75,6746.33");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5353.07,6536.96");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5727.80,6533.30");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1516.84,6363.01");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1919.12,6415.86");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3747.05,6397.17");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4099.32,6373.56");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4711.73,6426.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=285.51,6245.55");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=780.49,6181.04");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1242.34,6069.18");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1887.60,6034.76");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2398.01,6203.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2897.13,6209.56");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=3449.51,6177.85");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=5068.96,6261.19");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5409.56,6029.52");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5958.54,6257.10");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1580.91,5821.99");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2900.14,5801.08");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3864.88,5841.21");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4477.87,6014.44");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5957.71,5782.15");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=291.11,5677.84");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=703.35,5601.26");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=1341.33,5528.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1972.67,5553.95");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2357.93,5617.30");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4317.50,5609.05");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4924.93,5695.28");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5426.96,5608.49");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=151.19,5342.26");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=3313.22,5507.66");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3929.14,5475.97");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=5137.23,5343.44");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=529.24,5196.96");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=869.67,5093.62");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1276.73,5036.38");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1626.28,5071.96");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2363.72,5128.96");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2898.42,5045.12");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=4142.62,5104.93");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=5504.78,5032.27");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5925.64,5089.64");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=1912.00,4862.33");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2528.57,4795.85");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3315.65,4879.93");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3735.01,5025.86");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4390.82,4811.90");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=4704.37,5015.28");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=452.62,4613.02");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3708.72,4657.30");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4930.72,4624.37");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5276.91,4706.64");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=5723.83,4716.79");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=932.55,4432.41");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1360.60,4525.57");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1709.66,4484.80");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2260.20,4373.69");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2703.84,4470.09");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3184.89,4432.23");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4056.48,4403.28");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5987.14,4378.72");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=49.33,4092.26");
    }

}

