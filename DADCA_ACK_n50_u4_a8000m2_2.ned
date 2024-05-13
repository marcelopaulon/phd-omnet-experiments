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

network DADCA_ACK_n50_u4_a8000m2_2
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
        @display("i=misc/sensor;p=153.87,7935.76");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=642.20,7950.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1265.12,7811.56");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1682.26,7936.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2178.54,7872.34");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5095.43,7950.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=48.03,7574.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=920.63,7534.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3257.21,7523.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4174.00,7522.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4750.97,7599.09");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5310.40,7630.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5656.69,7567.02");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1875.09,7366.59");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2634.79,7501.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3846.58,7384.97");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4522.48,7258.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5061.40,7304.23");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=551.95,7163.47");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1367.10,7229.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2276.89,7142.95");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2920.61,7241.62");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5461.74,7184.75");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5805.80,7116.86");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=995.85,6915.63");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1826.07,6806.26");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2624.18,6925.20");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3353.69,6881.69");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3842.96,6778.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4337.57,6902.97");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4920.50,6947.32");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5832.69,6767.65");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=306.95,6629.54");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=641.76,6759.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2973.47,6597.52");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5366.07,6725.78");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1309.54,6355.75");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2438.82,6509.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3550.56,6330.69");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4391.33,6459.50");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5025.07,6327.71");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=186.65,6170.09");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=796.83,6261.24");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1672.16,6059.13");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2121.68,6132.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2628.73,6150.88");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3113.02,6234.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3869.09,6024.59");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5688.69,6193.09");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=645.05,5911.20");
    }

}

