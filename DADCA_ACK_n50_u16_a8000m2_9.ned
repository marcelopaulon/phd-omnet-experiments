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

network DADCA_ACK_n50_u16_a8000m2_9
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
        @display("i=misc/sensor;p=1252.17,7904.03");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1686.95,7856.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2618.54,7971.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3097.29,7797.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4501.10,7775.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4943.14,7911.41");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5737.42,7812.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=30.97,7625.79");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=664.47,7632.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2013.59,7679.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3807.66,7748.35");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5196.50,7662.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=424.15,7331.01");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1307.90,7368.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1718.24,7476.53");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2664.08,7408.45");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3332.36,7437.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3845.21,7327.38");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=791.59,7194.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2278.49,7227.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3037.35,7158.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3574.82,7093.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4241.71,7031.23");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4804.76,7217.25");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5563.90,7200.20");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=457.02,6832.81");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1337.63,6859.65");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1766.96,6822.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2354.44,6885.88");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3265.86,6856.66");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4735.84,6801.03");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5391.90,6861.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=36.20,6600.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2877.96,6527.77");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4379.08,6662.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=358.75,6429.49");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2020.52,6448.68");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2455.38,6345.63");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=3613.96,6395.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1090.48,6214.68");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1540.97,6244.51");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3337.20,6065.59");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4172.40,6196.63");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4621.48,6027.08");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4910.61,6259.05");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=105.87,5877.01");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=465.42,6010.49");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1633.37,5777.22");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2304.13,6015.04");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3912.07,5893.44");
    }

}

