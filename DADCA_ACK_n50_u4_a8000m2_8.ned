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

network DADCA_ACK_n50_u4_a8000m2_8
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
        @display("i=misc/sensor;p=466.06,7841.65");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1439.78,7930.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2350.24,7895.93");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2894.17,7853.30");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3430.26,7912.27");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4081.36,7765.37");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4776.39,7819.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1135.03,7708.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2629.02,7621.38");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5431.05,7594.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5801.16,7735.71");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=705.77,7287.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1674.30,7403.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2101.49,7400.44");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3454.15,7266.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3930.49,7272.46");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4589.35,7490.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=162.25,7169.12");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1357.64,7045.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1711.58,7029.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2352.67,7057.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2989.27,7068.71");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4277.82,7175.14");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4707.77,7141.24");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5043.82,7245.22");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5436.65,7184.22");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5889.80,7255.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=763.08,6878.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3823.61,6872.67");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4408.33,6793.53");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4926.82,6860.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=351.50,6524.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1147.62,6758.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1429.42,6547.61");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2701.05,6549.78");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3381.29,6571.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5394.50,6703.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1109.13,6335.86");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2213.41,6455.90");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4006.97,6385.07");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4886.34,6340.85");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5949.84,6388.06");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=760.67,6264.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1844.33,6078.82");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2796.98,6051.56");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3191.34,6207.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5212.24,6166.72");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=277.20,5887.44");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1201.39,5822.97");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2398.71,6001.64");
    }

}

