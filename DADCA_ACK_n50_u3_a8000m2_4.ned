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

network DADCA_ACK_n50_u3_a8000m2_4
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
        @display("i=misc/sensor;p=771.93,7958.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1131.87,7802.41");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3038.23,7998.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3499.56,7856.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4508.32,7925.43");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5195.08,7970.35");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5827.01,7874.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=377.44,7729.15");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2012.83,7655.03");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2718.75,7700.39");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3196.16,7569.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4099.69,7584.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4718.95,7527.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5087.54,7612.77");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1069.30,7422.75");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1572.14,7370.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5431.87,7467.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5795.78,7312.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=298.85,7192.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=681.33,7250.34");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2037.41,7166.61");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2723.24,7247.65");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3319.42,7197.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3769.51,7215.46");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4471.75,7081.99");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5030.35,7075.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=586.34,6836.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1011.69,6846.16");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2529.03,6776.96");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3023.37,6931.87");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5536.85,6955.19");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=110.79,6634.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1767.32,6699.86");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2240.99,6526.21");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3855.32,6673.73");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=615.34,6408.30");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=980.98,6343.96");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3537.75,6467.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4179.95,6456.93");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4541.72,6390.98");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5181.55,6352.50");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5560.60,6449.47");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1411.36,6064.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1779.20,6125.36");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2506.73,6103.92");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3137.63,6182.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3791.69,6026.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=204.66,5891.68");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=837.71,6000.97");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3445.01,5801.88");
    }

}

