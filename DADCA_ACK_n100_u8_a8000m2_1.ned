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

network DADCA_ACK_n100_u8_a8000m2_1
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
        @display("i=misc/sensor;p=759.57,7886.19");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=5304.99,7966.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1206.27,7614.96");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1841.10,7649.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2675.52,7702.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3326.38,7518.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4070.41,7704.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5557.07,7718.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5940.21,7718.76");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=870.13,7434.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2230.89,7409.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2574.27,7259.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3683.75,7493.99");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4035.49,7300.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5935.80,7309.64");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=66.51,7061.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1166.02,7081.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1606.16,7122.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1964.12,7066.57");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3074.95,7035.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5473.94,7219.16");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=690.37,6965.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3613.55,6863.54");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5907.45,6910.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=80.81,6604.17");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=409.59,6744.86");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=765.92,6534.63");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1503.38,6630.83");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2623.42,6701.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2991.47,6637.09");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4061.91,6683.69");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5037.88,6607.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5458.54,6729.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=287.86,6299.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1136.71,6424.90");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2119.14,6300.76");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2722.22,6273.16");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3602.18,6452.52");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4755.38,6383.67");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5902.44,6415.75");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=22.52,6053.74");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=578.05,6093.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=937.79,6093.00");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1641.54,6090.90");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3322.53,6163.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4253.77,6187.78");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5294.95,6097.38");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5679.46,6020.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1279.63,5952.73");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2015.67,5822.67");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2914.31,5793.11");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3431.29,5793.69");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4756.12,5990.33");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=155.26,5646.60");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=608.34,5706.20");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=947.51,5577.80");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=1427.85,5619.14");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=2484.78,5603.85");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=3962.98,5772.04");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=4382.48,5724.90");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=4794.16,5609.96");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5245.05,5648.72");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5593.52,5578.74");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1946.97,5320.58");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2755.33,5280.80");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3212.52,5301.16");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=3696.87,5365.86");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=360.16,5220.08");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1020.38,5146.67");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1537.70,5270.09");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4383.50,5272.11");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=4806.12,5167.92");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=28.10,4812.84");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=1508.00,4870.60");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=1947.16,4826.86");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=2470.57,5010.20");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2987.47,4947.60");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3783.30,4916.03");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4641.49,4808.45");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=5954.71,4989.14");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=672.88,4621.25");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2408.74,4592.96");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2780.05,4664.14");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3350.34,4739.86");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3681.23,4549.56");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5193.67,4589.20");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1390.81,4485.41");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1738.94,4434.44");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4278.42,4329.62");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5876.99,4302.41");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=306.69,4055.58");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1506.78,4086.23");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=2073.74,4186.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=2453.45,4240.40");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=2922.27,4204.68");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=3864.08,4194.23");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4963.44,4279.90");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=5436.03,4086.59");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=728.58,3975.89");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4589.31,3887.60");
    }

}

