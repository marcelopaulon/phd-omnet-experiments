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

network DADCA_ACK_n50_u8_a8000m2_5
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
        @display("i=misc/sensor;p=341.07,7788.58");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1139.18,7775.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1586.50,7960.72");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2121.31,7799.83");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3027.20,7971.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3506.77,7826.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4647.50,7845.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5028.77,7875.33");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5746.81,7982.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2695.72,7735.82");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4117.55,7687.62");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5211.33,7573.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=101.09,7502.40");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=895.33,7377.13");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1382.15,7484.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1868.73,7396.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2967.49,7369.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3301.89,7257.56");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4609.92,7266.60");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5699.08,7494.33");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1594.47,7144.29");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2202.15,7246.15");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3609.71,7078.94");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4068.39,7167.37");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5043.56,7133.30");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5989.97,7071.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=391.74,6895.84");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1133.41,6830.94");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1820.24,6784.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2529.08,6910.40");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5371.63,6881.94");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=670.75,6655.03");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3044.04,6752.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3470.75,6576.94");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3935.24,6557.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4520.66,6655.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4870.17,6555.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5607.24,6561.71");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5969.86,6595.22");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=26.14,6429.14");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1145.96,6352.70");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2218.39,6283.76");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2652.87,6510.44");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5213.08,6410.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=19.18,6039.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=550.18,6022.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1547.40,6200.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3338.56,6072.94");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3899.58,6172.18");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4452.58,6102.99");
    }

}

