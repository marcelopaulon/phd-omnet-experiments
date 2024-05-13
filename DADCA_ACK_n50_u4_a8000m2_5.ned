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

network DADCA_ACK_n50_u4_a8000m2_5
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
        @display("i=misc/sensor;p=572.90,7957.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1795.37,7771.91");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3441.39,7951.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4262.97,7930.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4783.51,7837.83");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5319.08,7810.22");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1200.94,7641.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3431.71,7560.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3951.31,7512.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5809.65,7582.13");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=186.09,7459.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=778.27,7432.30");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1125.17,7261.56");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1751.25,7420.58");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2399.42,7427.70");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2910.02,7280.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4681.60,7412.22");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5178.57,7270.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=445.93,7043.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4098.52,7136.22");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4719.22,7048.06");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5691.47,7132.02");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=116.59,6900.06");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=849.62,6869.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1350.94,6882.57");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2374.43,6957.03");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3162.74,7000.77");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5379.42,6935.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5891.92,6774.24");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=133.04,6517.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1160.28,6568.75");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1802.28,6711.70");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2465.76,6601.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4321.42,6596.66");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=823.77,6416.09");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1532.65,6439.71");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2064.96,6313.05");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3001.19,6493.62");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3809.97,6505.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4695.36,6504.65");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5123.74,6407.69");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5639.18,6487.29");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5988.17,6348.85");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1376.30,6079.55");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3133.66,6091.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3573.25,6198.52");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4267.95,6211.90");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4861.57,6141.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=477.28,5847.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=922.15,5815.95");
    }

}

