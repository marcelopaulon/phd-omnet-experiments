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

network DADCA_ACK_n50_a8000m2_0
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
        @display("i=misc/sensor;p=1032.90,7905.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1995.04,7842.50");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2947.26,7992.31");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3467.25,7982.03");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4087.72,7838.49");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4633.98,7831.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5095.16,7929.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5612.28,7818.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=466.28,7668.60");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1665.68,7641.49");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2707.33,7550.24");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=401.23,7303.90");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=867.76,7267.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2177.68,7495.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3040.36,7351.65");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3754.81,7497.49");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4722.06,7447.08");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1397.57,7121.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1814.34,7141.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2218.11,7082.21");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4372.69,7196.57");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4691.80,7033.76");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5682.68,7134.82");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=498.59,6905.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2679.24,6794.62");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3262.65,6982.06");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3780.68,6977.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=972.06,6520.00");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1586.56,6710.48");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4081.04,6679.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4703.77,6632.49");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5263.53,6654.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5920.25,6746.35");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=295.32,6376.40");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1929.98,6462.77");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2433.88,6408.37");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2889.57,6502.49");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3330.50,6433.91");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3768.24,6354.34");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=697.16,6181.27");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1072.19,6099.79");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1683.45,6096.37");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2134.11,6174.67");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3264.80,6071.93");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4351.62,6143.08");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4872.42,6055.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5533.64,6263.69");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=139.36,5790.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2752.95,5810.80");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5456.40,5910.67");
    }

}

