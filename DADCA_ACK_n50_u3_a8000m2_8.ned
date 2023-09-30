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

network DADCA_ACK_n50_u3_a8000m2_8
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
        @display("i=misc/sensor;p=133.75,7919.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=741.67,7810.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1309.45,7756.36");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3070.02,7783.21");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3434.08,7952.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5339.71,7774.79");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1668.74,7736.57");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2736.95,7646.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4662.65,7685.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5953.72,7667.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=457.90,7352.67");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1012.79,7431.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1910.69,7453.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3621.64,7393.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3988.41,7394.89");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4879.35,7402.60");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5707.44,7258.54");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1676.64,7162.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2512.82,7250.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2958.53,7060.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4398.82,7067.56");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5311.34,7114.13");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=831.34,7007.14");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1297.23,6910.61");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2048.98,6882.32");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2552.52,6892.56");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3611.38,6965.59");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3965.47,6987.07");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4775.34,6980.35");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5959.51,6768.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=433.90,6752.02");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1203.55,6557.08");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3321.88,6761.39");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3841.37,6658.31");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4190.52,6599.98");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5239.22,6750.40");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=784.32,6509.59");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1747.74,6467.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2168.20,6485.31");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2892.33,6323.11");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3580.96,6308.83");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4531.69,6424.13");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4918.68,6458.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5446.55,6378.87");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=141.18,6144.36");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=541.38,6104.87");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1094.50,6168.16");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4825.49,6037.11");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5789.07,6156.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1513.41,5977.49");
    }

}

