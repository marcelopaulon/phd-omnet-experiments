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

network DADCA_ACK_n50_u2_a8000m2_6
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
        @display("i=misc/sensor;p=88.52,7822.13");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=494.10,7997.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1982.47,7868.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2339.31,7981.06");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=868.36,7513.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1659.46,7566.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2683.25,7582.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4597.57,7695.41");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5145.51,7554.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5601.29,7616.11");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=494.50,7397.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1279.25,7496.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2122.84,7485.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2410.90,7280.78");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3076.75,7296.79");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3574.28,7318.93");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4071.48,7282.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4781.08,7385.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1017.03,7175.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1406.26,7041.31");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4454.93,7234.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4864.43,7023.81");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=242.36,6910.21");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=702.18,6845.02");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1847.86,6992.95");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2571.10,6931.93");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3251.70,6806.08");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3885.82,6956.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4402.25,6812.54");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5438.47,6941.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1064.38,6661.41");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2186.26,6690.54");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2785.20,6599.46");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4720.65,6596.24");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5590.02,6589.93");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=633.97,6314.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1625.52,6341.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3587.73,6428.54");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4229.37,6358.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5061.78,6383.80");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=245.72,6107.40");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2214.65,6196.25");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2603.07,6146.51");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3264.78,6182.85");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3961.28,6099.37");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5457.95,6048.70");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=716.37,5817.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1184.95,5904.34");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2603.73,5778.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2954.72,5990.77");
    }

}

