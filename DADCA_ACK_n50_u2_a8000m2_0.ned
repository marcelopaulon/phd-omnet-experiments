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

network DADCA_ACK_n50_u2_a8000m2_0
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
        @display("i=misc/sensor;p=148.96,7851.21");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=839.00,7946.39");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1293.64,7909.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2358.65,7770.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2971.70,7874.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3601.76,7937.67");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4634.17,7840.94");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1428.76,7523.99");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1867.76,7664.63");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3412.96,7586.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3873.84,7560.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5043.93,7701.14");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5762.51,7541.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=579.06,7456.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2770.68,7398.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=262.47,7079.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1141.02,7254.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1446.59,7061.55");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1877.47,7224.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2177.98,7013.32");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2630.32,7027.72");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3462.07,7096.97");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4693.59,7151.53");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5043.04,7214.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5789.12,7168.68");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1786.31,6879.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3876.16,6965.71");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5141.96,6825.41");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5513.18,6900.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5912.25,6818.87");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=162.51,6654.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=652.14,6660.81");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1341.45,6720.85");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4209.13,6651.46");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4767.76,6661.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=512.25,6322.73");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=954.66,6342.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1817.18,6391.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2470.11,6497.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3274.50,6477.59");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3724.69,6427.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5347.21,6343.75");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5998.60,6364.27");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=97.75,6187.00");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1400.58,6249.90");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2853.11,6234.10");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3249.99,6086.82");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4253.07,6242.26");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4733.43,6126.84");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=186.53,5829.48");
    }

}

