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

network DADCA_ACK_n50_u16_a8000m2_4
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
        @display("i=misc/sensor;p=947.14,7895.69");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1509.12,7991.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1996.00,7760.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3064.11,7818.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3501.20,7891.01");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3953.45,7903.76");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=585.05,7710.71");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1254.47,7727.80");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2514.94,7653.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4839.79,7645.70");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=36.37,7263.54");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=942.39,7430.81");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1528.59,7456.12");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3244.68,7389.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3635.95,7340.13");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4312.04,7376.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=381.97,7168.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1200.68,7129.98");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2075.80,7114.04");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2977.44,7089.67");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4557.18,7071.87");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5178.44,7061.27");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=45.78,6801.26");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=633.25,6781.88");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1538.20,6934.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2677.06,6827.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3378.45,6767.03");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4099.94,6899.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5831.16,6955.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1974.56,6604.94");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2335.52,6685.68");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5434.77,6754.29");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=292.12,6338.71");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=777.11,6330.82");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1193.16,6429.32");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2957.80,6485.91");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3592.16,6447.94");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4678.22,6514.13");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5148.81,6427.44");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5947.13,6387.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1450.83,6131.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2070.92,6175.83");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3169.51,6042.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4062.40,6177.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4604.69,6098.15");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5467.68,6082.47");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=365.61,5822.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=910.91,5997.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2698.36,5986.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3515.94,5848.83");
    }

}

