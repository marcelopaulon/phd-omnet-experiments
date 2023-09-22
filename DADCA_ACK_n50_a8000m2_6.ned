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

network DADCA_ACK_n50_a8000m2_6
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
        @display("i=misc/sensor;p=892.32,7801.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2053.09,7994.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2587.25,7991.50");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4161.62,7876.80");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4792.92,7770.17");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=548.82,7521.23");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1517.66,7571.99");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1956.17,7483.64");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2381.76,7570.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3055.72,7737.68");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3463.83,7603.90");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4115.36,7486.11");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5474.17,7639.19");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5966.72,7574.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2986.68,7340.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4715.09,7253.70");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5320.31,7230.99");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1019.23,7176.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1542.20,7195.30");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2456.54,7173.62");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3602.87,7120.35");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5038.87,6995.68");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5630.74,7008.58");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=631.13,6875.53");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1089.06,6811.70");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1666.31,6707.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2057.10,6813.15");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2947.38,6764.42");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4046.78,6825.52");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4497.46,6823.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5998.98,6866.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=207.04,6687.23");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2310.91,6520.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3423.22,6702.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4913.96,6539.60");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5489.09,6592.34");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=818.04,6325.06");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1385.96,6207.90");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1936.91,6309.49");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2899.24,6334.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3730.62,6415.01");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5929.11,6353.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2312.18,6128.17");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3295.92,6182.62");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4639.44,6115.68");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5143.42,6147.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5595.70,6169.78");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1929.86,5821.54");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2745.14,5753.73");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3408.76,5819.74");
    }

}

