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

network DADCA_ACK_n50_u8_a8000m2_7
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
        @display("i=misc/sensor;p=1016.83,7956.53");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2510.37,7836.44");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3140.45,7936.43");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4018.52,7873.39");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4516.56,7968.48");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5601.67,7909.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3448.59,7508.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4036.66,7507.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5914.29,7635.48");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=293.60,7294.09");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=767.41,7475.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1433.86,7400.58");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2218.14,7339.94");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2558.14,7448.86");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2899.52,7365.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4894.18,7428.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5560.15,7486.15");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1824.19,7047.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3327.33,7179.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3689.38,7195.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4202.18,7047.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4586.70,7218.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4963.06,7080.44");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5383.24,7089.14");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5948.23,7094.90");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=124.52,6941.01");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=949.46,6905.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1324.67,6917.97");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2481.15,6815.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3014.92,6777.71");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3645.70,6792.90");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=652.61,6619.67");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1503.77,6553.32");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2075.82,6749.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4552.69,6571.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5407.15,6706.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=202.90,6340.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1065.60,6414.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1994.49,6323.92");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2666.14,6381.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3139.69,6390.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3576.40,6330.81");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4037.24,6479.80");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5155.82,6363.84");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5958.77,6434.21");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=642.75,6169.95");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1672.34,6140.86");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4342.28,6217.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1161.41,5957.98");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2134.59,5876.14");
    }

}

