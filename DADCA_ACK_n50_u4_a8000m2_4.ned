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

network DADCA_ACK_n50_u4_a8000m2_4
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
        @display("i=misc/sensor;p=227.96,7921.33");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=606.14,7952.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1132.64,7863.28");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1560.48,7889.87");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2948.68,7911.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3468.96,7789.25");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3992.33,7903.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4978.47,7998.96");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=251.78,7555.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=717.54,7580.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1860.82,7589.78");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2469.81,7565.88");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4348.00,7706.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4776.55,7556.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5627.44,7620.40");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=469.99,7278.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1026.22,7395.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1517.78,7426.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2884.50,7499.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4011.87,7424.26");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4474.72,7279.19");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5180.56,7371.90");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1764.53,7031.00");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2088.67,7206.91");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2826.21,7101.12");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3175.91,7221.14");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3706.31,7158.09");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5934.20,7040.35");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=550.93,6923.26");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=878.57,6771.34");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1242.84,6767.71");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2259.44,6844.21");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4560.53,6900.29");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4923.80,6972.17");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5403.89,6896.61");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=250.02,6548.87");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=610.87,6521.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1844.04,6554.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3284.57,6673.82");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3914.69,6727.37");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4494.33,6519.51");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1046.32,6325.31");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2443.52,6406.09");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3652.46,6412.40");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5514.77,6474.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5912.54,6400.02");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1470.79,6086.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2246.81,6020.87");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3120.55,6084.67");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4120.92,6167.83");
    }

}

