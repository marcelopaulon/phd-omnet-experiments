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

network DADCA_ACK_n50_u8_a8000m2_8
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
        @display("i=misc/sensor;p=2668.63,7944.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3737.63,7842.75");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1087.73,7699.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2091.15,7710.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3403.38,7676.90");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4459.22,7678.91");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4901.39,7715.81");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=210.10,7461.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=807.98,7330.85");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1576.90,7502.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2573.27,7314.69");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3099.73,7401.53");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3443.64,7319.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4303.95,7352.07");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5357.91,7492.12");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5974.33,7476.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1096.15,7058.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2236.37,7028.89");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3930.38,7253.56");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4929.35,7194.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5423.83,7024.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5877.65,7091.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=304.38,6933.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=698.41,6802.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1776.93,6959.04");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3068.77,6967.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3606.98,6891.68");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4461.72,6870.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1825.59,6545.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2778.04,6759.93");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3301.56,6698.40");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4078.61,6694.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5037.28,6634.43");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5687.67,6761.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=759.18,6351.07");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1281.56,6444.22");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2501.44,6466.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2862.43,6363.15");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3257.47,6301.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3691.51,6290.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4111.69,6312.50");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4506.61,6506.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=135.93,6121.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2140.93,6075.06");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2626.59,6022.55");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4902.30,6046.36");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5339.03,6153.89");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5953.05,6125.48");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=507.43,5816.55");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1183.60,6018.11");
    }

}

