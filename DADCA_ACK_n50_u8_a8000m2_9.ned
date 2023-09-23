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

network DADCA_ACK_n50_u8_a8000m2_9
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
        @display("i=misc/sensor;p=3467.95,7764.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4161.94,7863.70");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4629.34,7773.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5411.32,7901.71");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5990.06,7920.55");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=873.89,7606.59");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1436.14,7519.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2431.21,7530.23");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2957.95,7673.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4990.50,7734.84");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=69.94,7338.00");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=583.03,7395.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2094.12,7342.84");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3396.00,7268.15");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4250.29,7410.23");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4756.03,7383.92");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5225.55,7293.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5577.69,7399.14");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1115.28,7058.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1789.33,7148.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2757.15,7242.94");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4950.15,7074.37");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=102.28,6850.20");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=638.05,6997.32");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1484.33,6850.51");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2282.61,6790.37");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2644.61,6908.42");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3853.22,6989.06");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4300.83,6879.12");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5257.16,6793.37");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5638.56,6880.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1851.59,6684.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3071.02,6762.47");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3452.21,6593.29");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3849.46,6627.97");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4565.61,6609.39");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4920.12,6597.20");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5941.04,6582.01");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=881.27,6442.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1426.20,6489.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1752.59,6330.25");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2319.30,6358.82");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2818.24,6348.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5281.32,6380.70");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5722.83,6304.79");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=275.98,6234.62");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=952.16,6065.79");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1310.07,6032.67");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1921.06,6020.91");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3314.76,6143.61");
    }

}

