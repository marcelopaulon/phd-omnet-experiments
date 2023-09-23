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
        @display("i=misc/sensor;p=775.82,7761.62");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1246.67,7958.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1697.60,7939.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2208.02,7831.96");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4021.30,7928.89");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4493.53,7796.12");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1885.55,7612.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2565.91,7534.45");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3458.30,7750.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5586.93,7698.74");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=448.72,7432.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1125.59,7496.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2904.03,7387.21");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3898.77,7388.10");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5772.02,7395.00");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=187.28,7021.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=904.44,7094.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1318.94,7083.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1786.75,7160.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2338.03,7173.37");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3270.26,7129.76");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4657.51,7235.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5329.53,7145.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5859.01,7019.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3088.83,6808.80");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4176.52,6949.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4999.86,6909.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=140.26,6602.21");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=640.86,6671.06");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1258.92,6599.49");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1901.66,6686.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2571.20,6710.01");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3668.78,6564.83");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4385.17,6577.69");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5316.57,6733.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=466.07,6282.50");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2697.54,6369.12");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3155.49,6350.47");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4099.84,6318.52");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4759.81,6462.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5792.49,6427.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1037.51,6122.26");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1711.05,6131.75");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2304.97,6211.23");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5420.22,6154.23");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2269.34,5824.90");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2654.44,5937.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3229.88,5857.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4626.79,5871.39");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5074.86,5996.90");
    }

}

